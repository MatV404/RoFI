#include "simplesim/distributor.hpp"

#include "simplesim/modules_communication.hpp"


using namespace rofi::simplesim;

Distributor::Distributor( gazebo::transport::Node & node,
                          ModulesCommunication & modulesCommunication,
                          bool verbose )
        : _modulesCommunication( modulesCommunication ), _logger( verbose )
{
    _pub = node.Advertise< rofi::messages::DistributorResp >( "~/distributor/response" );
    if ( !_pub ) {
        throw std::runtime_error( "Publisher could not be created" );
    }
    _sub = node.Subscribe( "~/distributor/request", &Distributor::onRequestCallback, this );
    if ( !_sub ) {
        throw std::runtime_error( "Subcriber could not be created" );
    }
}

void Distributor::onRequest( const rofi::messages::DistributorReq & req )
{
    using rofi::messages::DistributorReq;

    switch ( req.reqtype() ) {
        case DistributorReq::NO_REQ:
        {
            break;
        }
        case DistributorReq::GET_INFO:
        {
            if ( req.rofiid() != 0 ) {
                std::cerr << "Got GET_INFO distributor request with non-zero id\n";
            }
            sendResponse( onGetInfoReq() );
            break;
        }
        case DistributorReq::LOCK_ONE:
        {
            if ( req.rofiid() != 0 ) {
                std::cerr << "Got LOCK_ONE distributor request with non-zero id\n";
            }

            sendResponse( onLockOneReq( req.sessionid() ) );
            break;
        }
        case DistributorReq::TRY_LOCK:
        {
            sendResponse( onTryLockReq( req.rofiid(), req.sessionid() ) );
            break;
        }
        case DistributorReq::UNLOCK:
        {
            sendResponse( onUnlockReq( req.rofiid(), req.sessionid() ) );
            break;
        }
        default:
        {
            std::cerr << "Unknown distributor request type: " << req.reqtype() << "\n";
            break;
        }
    }
}

rofi::messages::DistributorResp Distributor::onGetInfoReq()
{
    rofi::messages::DistributorResp resp;
    resp.set_resptype( rofi::messages::DistributorReq::GET_INFO );

    _modulesCommunication.forEachLockedModule(
            [ &resp ]( ModuleId moduleId, const std::string & topic ) {
                auto & info = *resp.add_rofiinfos();
                info.set_rofiid( moduleId );
                info.set_lock( true );
                info.set_topic( topic );
            } );

    _modulesCommunication.forEachFreeModule( [ &resp ]( ModuleId moduleId ) {
        auto & info = *resp.add_rofiinfos();
        info.set_rofiid( moduleId );
        info.set_lock( false );
        // TODO possibly add topic for free modules (atm doesn't exist)
    } );

    return resp;
}

rofi::messages::DistributorResp Distributor::onLockOneReq( SessionId sessionId )
{
    rofi::messages::DistributorResp resp;
    resp.set_resptype( rofi::messages::DistributorReq::LOCK_ONE );
    resp.set_sessionid( sessionId );

    auto freeId = _modulesCommunication.lockFreeModule();
    if ( !freeId ) {
        return resp;
    }

    auto & info = *resp.add_rofiinfos();
    info.set_rofiid( *freeId );
    info.set_lock( true );
    if ( auto topic = _modulesCommunication.getTopic( *freeId ) ) {
        info.set_topic( *topic );
    }

    return resp;
}

rofi::messages::DistributorResp Distributor::onTryLockReq( ModuleId moduleId, SessionId sessionId )
{
    rofi::messages::DistributorResp resp;
    resp.set_resptype( rofi::messages::DistributorReq::TRY_LOCK );
    resp.set_sessionid( sessionId );

    auto & info = *resp.add_rofiinfos();
    info.set_rofiid( moduleId );
    info.set_lock( _modulesCommunication.tryLockModule( moduleId ) );
    if ( auto topic = _modulesCommunication.getTopic( moduleId ) ) {
        info.set_topic( *topic );
    }

    return resp;
}

rofi::messages::DistributorResp Distributor::onUnlockReq( ModuleId moduleId, SessionId sessionId )
{
    rofi::messages::DistributorResp resp;
    resp.set_resptype( rofi::messages::DistributorReq::UNLOCK );
    resp.set_sessionid( sessionId );

    auto & info = *resp.add_rofiinfos();
    info.set_rofiid( moduleId );

    if ( !_modulesCommunication.isLocked( moduleId ) ) {
        std::cerr << "Got UNLOCK distributor request without id locked\n";
        return resp;
    }

    _modulesCommunication.unlockModule( moduleId );
    info.set_lock( false );

    return resp;
}
