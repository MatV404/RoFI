#include <networking/networkManager.hpp>
#include <lwip/udp.h>
#include <lwip++.hpp>

#include <atoms/util.hpp>

#include <set>
#include <chrono>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>

namespace rofi::leadership {

    using namespace rofi::net;
    using namespace std::chrono_literals;

    enum InvitationStatus : char {
        DOWN,
        ELECTION,
        NORMAL,
        REORGANIZATION,
    };

    enum InvitationMessage : char {
        ARE_YOU_THERE,
        ARE_YOU_THERE_RES,
        ARE_YOU_COORDINATOR,
        ARE_YOU_COORDINATOR_RES,
        INVITATION,
        READY,
        READY_RES, // Task succesfully received.
        ACCEPT,
        ACCEPT_RES, // It is necessary to confirm that a coordinator was able to register the accepted invitation. Otherwise, the coordinator could be down already.
    };

    class InvitationElection {
        struct GroupNumber {
            int coordinatorId;
            int sequenceNum;

            bool operator== ( const GroupNumber& rhs ) const {
                return coordinatorId == rhs.coordinatorId && sequenceNum == rhs.sequenceNum;
            }
        };

        struct Node {
            int id;
            const Ip6Addr address;

            bool operator< ( const Node& rhs ) const {
                return id < rhs.id;
            }

            Node( int id, Ip6Addr addr ) : id( id ), address( addr ) {}
        };
        
        int _id;
        InvitationStatus _nodeStatus;
        GroupNumber _groupNumber; //S(i).g
        int _groupCounter = 0; //S(i).counterF

        Ip6Addr _coordinator;
        const Ip6Addr& _myAddr;
        std::set< Ip6Addr > _up; //S(i).up
        std::set< Ip6Addr > _foundCoordinators;

        NetworkManager& _netmg;
        const std::string& _routing;
        udp_pcb* _pcb = nullptr;
        
        std::mutex _waitMutex; //_functionMutex;
        std::condition_variable _condVar;
        std::function< std::pair< void*, int >() > _calculateTask;
        std::function< void ( void*, int ) > _getTask;
        std::function< void () > _stopWork;

        u16_t _port;
        int _timeout;
        std::atomic< bool > _coordinatorContacted;

        Ip6Addr _awaited;
        bool _started = false;

        PBuf _composeMessage( InvitationMessage messageType ) {
            switch ( messageType ) {
                case InvitationMessage::ACCEPT_RES:
                case InvitationMessage::READY_RES:
                case InvitationMessage::ARE_YOU_COORDINATOR: {
                    auto packet = PBuf::allocate( sizeof( InvitationMessage ) );
                    as< InvitationMessage >( packet.payload() ) = messageType;
                    return packet;
                }

                case InvitationMessage::INVITATION: {
                    auto packet = PBuf::allocate( sizeof( InvitationMessage ) + sizeof( GroupNumber ) + sizeof( Ip6Addr ) );
                    as< InvitationMessage >( packet.payload() ) = messageType;
                    as< GroupNumber >( packet.payload() + sizeof( InvitationMessage ) ) = _groupNumber;
                    as< Ip6Addr >( packet.payload() + sizeof( InvitationMessage ) + sizeof( GroupNumber ) ) = _coordinator;
                    return packet;
                }
                case InvitationMessage::ARE_YOU_THERE:
                case InvitationMessage::ACCEPT: {
                    auto packet = PBuf::allocate( sizeof( InvitationMessage ) + sizeof( GroupNumber ) + sizeof( int ) );
                    as< InvitationMessage >( packet.payload() ) = messageType;
                    as< GroupNumber >( packet.payload() + sizeof( InvitationMessage ) ) = _groupNumber;
                    as< int >( packet.payload() + sizeof( InvitationMessage ) + sizeof( GroupNumber ) ) = _id;
                    return packet;
                }

                default: {
                    assert( false && "Incorrect message type received at _composeMessage\n");
                }
            }
        }

        PBuf _composeResponse( InvitationMessage messageType, bool answer ) {
            int size = sizeof( InvitationMessage ) + sizeof( bool );
            if ( messageType == InvitationMessage::ARE_YOU_COORDINATOR_RES ) {
                size += sizeof( int );
            }
            auto packet = PBuf::allocate( size );
            as< InvitationMessage >( packet.payload() ) = messageType;
            as< bool >( packet.payload() + sizeof( InvitationMessage ) ) = answer;
            if ( messageType == InvitationMessage::ARE_YOU_COORDINATOR_RES ) {
                as< int >( packet.payload() + sizeof( InvitationMessage ) + sizeof( bool ) ) = _id;
            }
            return packet;
        }

        PBuf _composeReadyMessage( ) {
            std::pair< void*, int > tasks = _calculateTask();
            PBuf packet = PBuf::allocate( sizeof( InvitationMessage ) + sizeof( GroupNumber ) + tasks.second );
            as< InvitationMessage >( packet.payload() ) = InvitationMessage::READY;
            as< GroupNumber >( packet.payload() + sizeof( InvitationMessage ) ) = _groupNumber;
            std::memcpy( ( packet.payload() + sizeof( InvitationMessage ) + sizeof( GroupNumber ) ), tasks.first, tasks.second );
            return packet;
        }

        bool _sendMessage( const Ip6Addr& ip, PBuf packet ){
            assert( _pcb && "PCB is null.\n" );
            Ip6Addr addr = ip;
            addr.zone = 0;
            err_t res = udp_sendto( _pcb, packet.release(), &addr, _port );
            assert( res != ERR_MEM && "Out of memory.\n" );
            return res == ERR_OK;
        }

        void _onAcceptMessage( const Ip6Addr addr, PBuf packet ) {
            GroupNumber group = as< GroupNumber >( packet.payload() + sizeof( InvitationMessage ) );
            int id = as< int >( packet.payload() + sizeof( InvitationMessage ) + sizeof( GroupNumber ) );
            if ( _nodeStatus == InvitationStatus::ELECTION 
                 && group == _groupNumber 
                 && _coordinator == _myAddr ) {
                auto res = _up.emplace( addr );
                _awaited = _myAddr;
                _sendMessage( addr, _composeMessage( InvitationMessage::ACCEPT_RES ) );
            }
        }

        void _onAcceptResponse( const Ip6Addr& addr ) {
            if ( _awaited != addr ) {
                _recovery();
                return;
            }
            _awaited = _myAddr;
            // _functionMutex.lock();
            _nodeStatus = InvitationStatus::REORGANIZATION;
            // _functionMutex.unlock();
        }

        void _onAreYouThere( const Ip6Addr& addr, GroupNumber groupNum ) {
            // _functionMutex.lock();
            auto res = std::find_if( _up.begin(), _up.end(), [ addr ]( const Ip6Addr& address ) { return address == addr; } );
            _sendMessage( addr, _composeResponse( InvitationMessage::ARE_YOU_THERE_RES, 
                                                  groupNum == _groupNumber 
                                                  && _myAddr == _coordinator 
                                                  && res != _up.end() ) );
            // _functionMutex.unlock();
        }

        void _onAreYouCoordinator( const Ip6Addr& addr ) {
            // _functionMutex.lock();
            if ( addr == _coordinator ) {
                _coordinatorContacted = true;
            }
            _sendMessage( addr, _composeResponse( InvitationMessage::ARE_YOU_COORDINATOR_RES, 
                                                          _nodeStatus == InvitationStatus::NORMAL && _myAddr == _coordinator ) );
            // _functionMutex.unlock();
        }

        void _onAreYouThereRes( const Ip6Addr& addr, PBuf packet ) {
            if ( addr != _awaited ) {
                return;
            }
            bool response = as< bool >( packet.payload() + sizeof( InvitationMessage ) );
            if ( response ) {
                _awaited = _myAddr;
            }
        }

        void _onAreYouCoordinatorRes( const Ip6Addr& addr, PBuf packet ) {
            if ( addr != _awaited ) {
                return;
            }
            bool response = as< bool >( packet.payload() + sizeof( InvitationMessage ) );
            int id = as< int >( packet.payload() + sizeof( InvitationMessage ) + sizeof( bool ) );
            if ( response ) {
                _foundCoordinators.emplace( addr );
            }
            _awaited = _myAddr;
        }

        void _onInvitation( const Ip6Addr& addr, GroupNumber group, const Ip6Addr& newCoordinator ) {
            if ( _nodeStatus != InvitationStatus::NORMAL ) {
                return;
            }
            _nodeStatus = InvitationStatus::ELECTION;
            _stopWork();
            bool wasCoordinator = _coordinator == _myAddr;
            std::set< Ip6Addr > tempUp = _up;
            _up.clear();

            _coordinator = newCoordinator;
            _groupNumber = group;

            if ( wasCoordinator ) {
                for ( const Ip6Addr& address : tempUp ) {
                    _sendMessage( address, _composeMessage( InvitationMessage::INVITATION ) );
                }
            }

            if ( !_sendMessage( newCoordinator, _composeMessage( InvitationMessage::ACCEPT ) ) ) {
                _recovery();
            }
            
            _awaited = newCoordinator;
            return;
        }

        void _onReadyMessage( const Ip6Addr& addr, PBuf packet ) {
            // _functionMutex.lock();
            GroupNumber group = as< GroupNumber >( packet.payload() + sizeof( InvitationMessage ) );
            if ( _nodeStatus == InvitationStatus::REORGANIZATION && _groupNumber == group ) {
                void* task = packet.payload() + sizeof( InvitationMessage ) + sizeof( GroupNumber );
                int size = packet.size() - ( sizeof( InvitationMessage ) + sizeof( GroupNumber ) ); 
                _getTask( task, size );
                _nodeStatus = InvitationStatus::NORMAL;
                _sendMessage( addr, _composeMessage( InvitationMessage::READY_RES ) );
            }
            // _functionMutex.unlock();
        }

        void _recovery() {
            // _functionMutex.lock();
            _nodeStatus = InvitationStatus::ELECTION;
            _stopWork();
            _groupCounter++;
            _groupNumber.sequenceNum = _groupCounter;
            _groupNumber.coordinatorId = _id;
            _coordinator = _myAddr;
            _up.clear();
            _up.emplace( _myAddr );
            _nodeStatus = InvitationStatus::REORGANIZATION;
            std::pair< void*, int > tasks = _calculateTask();
            _getTask( tasks.first, tasks.second );
            _nodeStatus = InvitationStatus::NORMAL;
            // _functionMutex.unlock();
        }

        void _mergeGroups() {
            // _functionMutex.lock();
            _nodeStatus = InvitationStatus::ELECTION;
            _stopWork();
            _groupCounter++;
            _groupNumber.sequenceNum = _groupCounter;
            _groupNumber.coordinatorId = _id;
            _coordinator = _myAddr;

            std::set< Ip6Addr > tempSet = _up; // ToDo: Use references.
            _up.clear();
            _up.emplace( _myAddr );
            // _functionMutex.unlock();
            for ( const Ip6Addr& coordinator : _foundCoordinators ) {
                _sendMessage( coordinator, _composeMessage( InvitationMessage::INVITATION ) );
            }
            for ( const Ip6Addr& address : tempSet ) {
                _sendMessage( address, _composeMessage( InvitationMessage::INVITATION ) );
            }

            // Wait ... some time.
            sleep( _timeout  );

            // _functionMutex.lock();
            _nodeStatus = InvitationStatus::REORGANIZATION;
            // _functionMutex.unlock();

            for ( const Ip6Addr& address : _up ) {
                _awaited = address;
                _sendMessage( address, _composeReadyMessage() );
                std::unique_lock< std::mutex > lock( _waitMutex );
                if ( !_condVar.wait_for( lock, 1000ms, [ this ] { return _awaited == _myAddr; } ) ) {
                    _awaited = _myAddr;
                    _recovery();
                    return;
                }
                lock.unlock();
            }

            // _functionMutex.lock();
            _nodeStatus = InvitationStatus::NORMAL;
            // _functionMutex.unlock();
        }

        void _checkForGroups() {
            _foundCoordinators.clear();
            if ( _nodeStatus == InvitationStatus::NORMAL && _coordinator == _myAddr ) {
                auto records = _netmg.routingTable().recordsLearnedFrom( *_netmg.getProtocol( _routing ) ); 
                for ( auto record : records ) {
                    auto addr = record.ip();
                    if ( addr == _myAddr ) {
                        continue;
                    }
                    addr.zone = 0;
                    _awaited = addr;
                    if ( !_sendMessage( addr, _composeMessage( InvitationMessage::ARE_YOU_COORDINATOR ) ) ) {
                        continue;
                    }
                    std::unique_lock< std::mutex > lock( _waitMutex );
                    if ( !_condVar.wait_for( lock, 1000ms, [ this ] { return _awaited == _myAddr; } ) ) {
                        continue;
                    }
                    lock.unlock();
                }
                _awaited = _myAddr; // Since we never send a message to ourselves, we can consider _myAddr as undefined for _awaited.
                if ( _foundCoordinators.empty() ) {
                    return;
                }

                Ip6Addr highestPrio = *_foundCoordinators.rbegin();
                if ( highestPrio > _myAddr ) {
                    sleep( _timeout ); // This is magical. Fix.
                    return;
                }
                _mergeGroups();
            }
        }

        void _checkCoordinator() {
            _awaited = _coordinator;
            if ( !_sendMessage( _coordinator , _composeMessage( InvitationMessage::ARE_YOU_THERE ) ) ) {
                _awaited = _myAddr; 
                _recovery();
                return;
            }

            std::unique_lock< std::mutex > lock( _waitMutex );
            if ( !_condVar.wait_for( lock, 1000ms, [ this ]{ return _awaited == _myAddr; } ) ) {
                _awaited = _myAddr; 
                _recovery();
            }
            lock.unlock();
            return;
        }

        void _periodicCheck() {
            while ( true ) {
                if ( _nodeStatus == InvitationStatus::NORMAL && _coordinator == _myAddr ) {
                    _checkForGroups();
                } 
                if ( _nodeStatus != InvitationStatus::DOWN && _coordinator != _myAddr ) {
                    if ( _coordinatorContacted ) {
                        _coordinatorContacted = false;
                    } else {
                        _checkCoordinator();
                    }
                }
                sleep( _timeout );
            }
        }
    public:
        InvitationElection( int id, Ip6Addr& myAddr, u16_t port, 
                            NetworkManager& netmg, const std::string& routingName, 
                            int timeout, std::function< std::pair< void*, int >() > calculateTask, 
                            std::function< void ( void*, int ) > getTask,
                            std::function< void () > stopWork ) 
        : _netmg( netmg ), _myAddr( myAddr), _coordinator( myAddr ), _routing( routingName ), _awaited( myAddr ) {
            _id = id;
            _nodeStatus = InvitationStatus::REORGANIZATION;
            _groupNumber.coordinatorId = _id;
            _groupNumber.sequenceNum = 0;
            _timeout = timeout;
            _port = port;
            _calculateTask = calculateTask;
            _getTask = getTask;
            _stopWork = stopWork;
        }

        bool setUp() {
            _pcb = udp_new();
            assert( _pcb && "PCB is null" );
            err_t bind = udp_bind( _pcb, IP6_ADDR_ANY, _port );
            assert( bind == ERR_OK && "Bind failed");
            udp_recv( _pcb, 
                        [ ] ( void* invCls, struct udp_pcb*, struct pbuf* p, const ip6_addr_t* addr, u16_t ) {
                            if ( !p || !addr ) {
                                return;
                            }
    
                            auto packet = PBuf::own( p );
                            as< InvitationElection >( invCls ).onMessage( Ip6Addr( *addr ), packet );
                    }, this ); // Here, we only want to pass the onMessage function, not the whole class.
            return true;
        }

        void start() {
            if ( _started ) {
                return;
            }
            _started = true;
            _recovery();
            std::thread thread{ [ this ]() {
                this->_periodicCheck();
            }};
            thread.detach();
        }

        /**
         * Returns the identity of the node's assumed leader, as well
         * as an indicator as to whether the leader is up to date ( second == true ), 
         * or whether the leader is currently being re-elected ( second == false ).
        */
        std::pair< Ip6Addr&, bool > getLeader() {
            return std::pair< Ip6Addr&, bool >( _coordinator, _nodeStatus == InvitationStatus::NORMAL );
        }

        /**
         * Retrieves the modules this current module believes are available.
        */
        const std::set< Ip6Addr > getUp() {
            return _up;
        }

        void onMessage( const Ip6Addr addr, PBuf packet ) {
            if ( _nodeStatus == InvitationStatus::DOWN ) {
                return;
            }
            InvitationMessage messageType = as< InvitationMessage >( packet.payload() );
            switch ( messageType ) {
                case InvitationMessage::ACCEPT:
                    _onAcceptMessage( addr, packet );
                    return;
                case InvitationMessage::ACCEPT_RES:
                    _onAcceptResponse( addr );
                    return;
                case InvitationMessage::ARE_YOU_COORDINATOR:
                    _onAreYouCoordinator( addr );
                    return;
                case InvitationMessage::ARE_YOU_COORDINATOR_RES:
                    _onAreYouCoordinatorRes( addr, packet );
                    return;
                case InvitationMessage::ARE_YOU_THERE:
                    _onAreYouThere( addr, as< GroupNumber >( packet.payload() + sizeof( InvitationMessage ) ) );
                    return;
                case InvitationMessage::ARE_YOU_THERE_RES:
                    _onAreYouThereRes( addr, packet );
                    return;
                case InvitationMessage::INVITATION: {
                    GroupNumber grpNum = as< GroupNumber >( packet.payload() + sizeof( InvitationMessage ) );
                    Ip6Addr newCoord = as< Ip6Addr >( packet.payload() + sizeof( InvitationMessage ) + sizeof( GroupNumber ) );
                    _onInvitation( addr, grpNum, newCoord );
                    return;
                }
                case InvitationMessage::READY: {
                    _onReadyMessage( addr, packet );
                    return;
                }
                case InvitationMessage::READY_RES: {
                    if ( _awaited != addr ) {
                        return;
                    }
                    _awaited = _myAddr;
                }
            }
            return;
        }

        void switchDown() {
            if ( _nodeStatus != InvitationStatus::DOWN ) {
                _nodeStatus = InvitationStatus::DOWN;
                _stopWork();
            } else {
                _recovery();
            }
        }
    };
}