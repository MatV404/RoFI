#pragma once

#include "lwip++.hpp"
#include <networking/protocol.hpp>
#include <networking/interface.hpp>
#include <networking/routingTable.hpp>

#include <atoms/util.hpp>

#include <vector>
#include <map>
#include <set>

namespace rofi::net {

/**
 * Proof-of-concept of a simple election protocol that restarts on changes in topology.
 * Made out of a merge of simple-reactive and leader-election provided in the initial network-manager protocol examples,
 * with some additional modification to ensure all functions well.
*/
class SimpleElection : public Protocol {
    std::vector< std::reference_wrapper< const Interface > > _managedInterfaces;
    std::set< std::string > _interfaceWithCb;

    using Update = std::pair< Protocol::Route, RoutingTable::Record >;
    std::vector< Update > _updates;

    using ConfigUpdate = std::pair< ConfigAction, ConfigChange >;
    std::vector< ConfigUpdate > _confChanges;
    std::map< Interface::Name, bool > _converged;

    Ip6Addr _leaderAddress;
    uint8_t _mask;
    int _leaderId;
    int _id;

    void _resetConverged( ) {
        for ( auto& [ key, _ ] : _converged ) {
            _converged[key] = false;
        }
    }

    void _resetElection( ) {
        if ( _leaderId == _id ) {
            _confChanges.push_back( { ConfigAction::REMOVE_IP, { "rl0", _leaderAddress, _mask } } );
        }

        for ( auto record : routingTableCB() ) {
            if ( record.ip() != _leaderAddress ) {
                continue;
            }

            for ( auto gateway : record.gateways() ) {
                _updates.push_back( { Route::RM, { record.ip(), record.mask(), gateway.name(), gateway.cost() } } );
            }
        }

        _resetConverged();
        _leaderId = _id;
    }

    bool _performElection( const std::string& interfaceName, int otherId ) {
        _converged[ interfaceName ] = otherId == _leaderId;

        if ( !_converged[ interfaceName ] ) {
            if ( otherId < _leaderId ) {
                if ( _leaderId == _id ) {
                    _confChanges.push_back( { ConfigAction::REMOVE_IP, { "rl0", _leaderAddress, _mask } } );
                }
                _leaderId = otherId;
                _resetConverged();
                return true;
            }

            if ( _id == _leaderId ) {
                _confChanges.push_back( { ConfigAction::ADD_IP, { "rl0", _leaderAddress, _mask } } );
                return true;
            }

            if ( otherId > _leaderId ) {
                _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
                return true;
            }
        }

        if ( _id == _leaderId ) {
                _confChanges.push_back( { ConfigAction::ADD_IP, { "rl0", _leaderAddress, _mask } } );
                return true;
        }
        
        return false;
    }

    PBuf _createMsg( const std::string& interfaceName ) {
        auto records = routingTableCB();

        int count = 0;
        for ( auto& r : records ) {
            auto g = r.best();
            if ( !g || ( g->name() == interfaceName && g->cost() != 0 ) )
                continue;
            count++;
        }

        auto packet = PBuf::allocate( 2 + count * ( Ip6Addr::size() + 3 ) + sizeof( int ) );
        as< uint16_t >( packet.payload() ) = static_cast< uint16_t >( count );
        auto* data = packet.payload() + 2;

        as< int >( data ) = _leaderId;
        data += sizeof( int );
        for ( const auto& r : records ) {
            auto g = r.best();
            if ( !g || ( g->name() == interfaceName && g->cost() != 0 ) )
                continue;

            as< Ip6Addr >( data ) = r.ip();
            as< uint8_t >( data + Ip6Addr::size() ) = r.mask();
            as< uint16_t >( data + Ip6Addr::size() + 1 ) = static_cast< uint16_t >( g->cost() );
            data += Ip6Addr::size() + 3;
        }

        return packet;
    }

    bool _addInterface( const Interface& interface ) {
        for ( auto [ ip, mask ] : interface.getAddress() ) {
            addAddressOn( interface, ip, mask );
        }

        return true;
    }

    bool _removeInterface( const Interface& interface ) {
        // remove all routes that has this interface as their gateway
        bool removed = false;
        bool changedTopo = false;
        auto records = routingTableCB();
        for ( const auto& rec : records ) {
            for ( const auto& g : rec.gateways() ) {
                if ( g.name() != interface.name() )
                    continue;

                if ( rec.size() == 1 ) {
                    changedTopo = true;
                }

                _updates.push_back( { Route::RM
                                    , { rec.ip(), rec.mask(), interface.name(), g.cost(), g.learnedFrom() } } );
                removed = true;
            }
        }

        if ( changedTopo ) {
            _resetElection();
        }

        return removed;
    }

public:
    SimpleElection( int id, const Ip6Addr& leaderAddr, uint8_t mask ) 
        : _leaderAddress( leaderAddr ), _mask( mask ), _id( id ) {
            _leaderId = _id;
    }

    virtual bool onMessage( const std::string& interfaceName, rofi::hal::PBuf packetWithHeader ) override {
        auto packet = PBuf::own( pbuf_free_header( packetWithHeader.release(), IP6_HLEN ) );
        int count = static_cast< int >( as< uint16_t >( packet.payload() ) );
        auto data = packet.payload() + 2;

        int otherId = as< int >( data );

        bool somethingNew = false;

        data += sizeof( int );

        bool topoChanged = false;

        auto records = routingTableCB();
        std::set< int > validRecords;

        for ( int i = 0; i < count; i++ ) {
            Ip6Addr ip  = as< Ip6Addr >( data );
            auto mask   = as< uint8_t >( data + Ip6Addr::size() );
            auto cost   = as< uint16_t >( data + Ip6Addr::size() + 1 ) + 1;
            RoutingTable::Record rec{ ip, mask, interfaceName, cost };
            // do we know about this route already?
            int index = -1;
            auto it = std::find_if( records.begin(), records.end(), [ &rec, this, &index ]( auto& r ) {
                index++;
                return r.compareNetworks( rec ) && r.contains( { rec.best()->name(), rec.best()->cost(), this } );
            } );
            if ( it == records.end() ) { // no, so we add it
                auto existing = std::find_if( records.begin(), records.end(), [ &rec, this ]( auto& r ) {
                    return r.compareNetworks( rec );
                } );
                if ( existing != records.end() ) {
                    topoChanged = true;
                }
                somethingNew = true;
                Update update{ Route::ADD, rec };
                _updates.push_back( update );
            } else {
                validRecords.insert( index );
            }

            data += Ip6Addr::size() + 3;
        }

        // remove those records, that were not used in the find above -- so they were not
        // among routes in the processed update message, and therefore are no longer valid
        for ( unsigned i = 0; i < records.size(); i++ ) {
            if ( validRecords.contains( i ) )
                continue;
            for ( auto& g : records[ i ].gateways() ) {
                if ( g.name() == interfaceName ) {
                    if ( records[ i ].size() == 1 ) {
                        topoChanged = true;
                    }
                    _updates.push_back( { Route::RM
                                        , { records[ i ].ip(), records[ i ].mask(), g.name(), g.cost(), this } } );
                    somethingNew = true;
                }
            }
        }

        if ( topoChanged ) {
            _resetElection( );
        }

        somethingNew = _performElection( interfaceName, otherId ) | somethingNew;


        return somethingNew;
    }

    virtual bool afterMessage( const Interface& i, std::function< void ( PBuf&& ) > f, void* /* args */ ) override {
        f( std::move( _createMsg( i.name() ) ) );
        return false;
    }

    virtual bool onInterfaceEvent( const Interface& interface, bool connected ) override {
        assert( manages( interface ) && "onInterfaceEvent within SimpleReactive got unmanaged interface" );

        bool res = false;
        if ( connected ) {
            res = _addInterface( interface );
            // add everything we know, to pass it to the new neighbour
            for ( auto& rec : routingTableCB() ) {
                if ( rec.best() && rec.best()->name() != interface.name() )
                    _updates.push_back( { Route::ADD
                                        , { rec.ip(), rec.mask(), rec.best()->name(), rec.best()->cost(), this } } );
            }
        } else {
            res = _removeInterface( interface );
        }

        return res;
    }

    virtual bool hasRouteUpdates() const override { return !_updates.empty(); }

    virtual std::vector< std::pair< Route, RoutingTable::Record > > getRouteUpdates() const override {
        return _updates;
    }

    virtual bool hasConfigUpdates() const override { return !_confChanges.empty(); }

    virtual std::vector< std::pair< ConfigAction, ConfigChange > > getConfigUpdates() const {
        return _confChanges;
    }


    virtual void clearUpdates() override {
        _updates.clear();
        _confChanges.clear();
    }

    virtual bool addAddressOn( const Interface& interface, const Ip6Addr& ip, uint8_t mask ) override {
        if ( !ip.linkLocal() ) {
            // add it without mentioning the origin as we want all others to propagate it
            _updates.push_back( { Route::ADD, { ip, mask, interface.name(), 0 } } );
            return true;
        }

        return false;
    }

    virtual bool rmAddressOn( const Interface& interface, const Ip6Addr& ip, uint8_t mask ) override {
        if ( !ip.linkLocal() ) {
            Update rec{ Route::RM, { ip, mask, interface.name(), 0 } };
            _updates.push_back( rec );
            return true;
        }

        return false;
    }

    virtual bool addInterface( const Interface& interface ) override {
        if ( manages( interface ) ) // interface is already managed
            return false;

        _managedInterfaces.push_back( std::ref( interface ) );
        bool result = _addInterface( interface );
        _converged[ interface.name() ] = false;
        return result;
    }

    virtual bool removeInterface( const Interface& interface ) override {
        auto it = std::find_if( _managedInterfaces.begin(), _managedInterfaces.end()
                              , [ &interface ]( const auto& i ) { return interface == i; } );

        if ( it == _managedInterfaces.end() )
            return false;

        std::swap( *it, _managedInterfaces.back() );
        _managedInterfaces.pop_back();

        return _removeInterface( interface );
    }

    virtual bool manages( const Interface& interface ) const override {
        return std::ranges::any_of( _managedInterfaces, [ &interface ]( const Interface& i ) {
            return interface == i;
        } );
    }

    virtual Ip6Addr address() const { return Ip6Addr( "ff02::c:c:c:ae" ); };
    virtual std::string name() const { return "simple-election"; };

};

} // namespace rofi::net