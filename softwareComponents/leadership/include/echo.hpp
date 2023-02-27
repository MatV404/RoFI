#pragma once

#include "lwip++.hpp"
#include <networking/protocol.hpp>
#include <networking/interface.hpp>
#include <networking/routingTable.hpp>

#include <atoms/util.hpp>

#include <vector>
#include <map>
#include <set>
#include <utility>

namespace rofi::net {
    class EchoElection : public Protocol {
        struct InterfaceReceived {
            bool regularReceived;
            bool leaderReceived;
            bool initiated;
        };

        enum MessageType {
            ELECTION_MESSAGE,  // Carries basic election messages 
            LEADER_MESSAGE,    // Announces leadership
            INITIATE_MESSAGE,  // For the initiation of the algorithm
        };

        enum RelayType {
            SEND_TO_NEIGHBORS, // All but parent
            SEND_TO_PARENT,    // Only parent
            SEND_TO_ALL,       // Everyone
            DO_NOT_SEND,       // Stops message sending
        };

        std::vector< std::reference_wrapper< const Interface > > _managedInterfaces;
        std::set< std::string > _interfaceWithCb;

        std::vector< std::pair< ConfigAction, ConfigChange > > _confChanges;

        Ip6Addr _leaderAddress;
        uint8_t _mask;
        int _id;
        
        int _leaderId;
        int _currentWaveId;
        Interface::Name _parent;

        MessageType _messageType;
        RelayType _relayType;
        bool _restart;

        std::map< Interface::Name, bool > _pendingDiscoveryResponse;
        std::map< Interface::Name, InterfaceReceived > _received;

        bool _noLeaderReceived() {
            for ( const auto& [ key, _ ] : _received ) {
                if ( _received[ key ].leaderReceived ) {
                    return false;
                }
            }
            return true;
        }

        bool _noPendingResponse() {
            for ( const auto& [ key, _ ] : _pendingDiscoveryResponse ) {
                if ( _pendingDiscoveryResponse[ key ] ) {
                    std::cout << key << "\n";
                    return false;
                }
            }
            return true;
        }

        bool _allRegularReceived() {
            for ( const auto& [ key, _ ] : _received ) {
                if ( !_received[ key ].regularReceived ) {
                    return false;
                }
            }
            return true;
        }

        void _resetReceived() {
            for ( const auto& [ key, _ ] : _received ) {
                _received[key].leaderReceived = false;
                _received[key].regularReceived = false;
            }
        }

        bool _allInitiated() {
            for ( const auto& [ _, value ] : _received ) {
                if ( !value.initiated ) {
                    return false;
                }
            }
            return true;
        }

        bool _onElectionMessage( const std::string& interfaceName, 
                                 int otherWaveId ) {
            if ( _restart && _id == _leaderId ) {
                _confChanges.push_back( { ConfigAction::REMOVE_IP, { "rl0", _leaderAddress, _mask } } );
            }

            if ( _currentWaveId < otherWaveId ) {
                if ( _restart && _currentWaveId == _id ) {
                    _relayType = RelayType::SEND_TO_NEIGHBORS;
                    _messageType = MessageType::ELECTION_MESSAGE;
                    _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                    return true;
                }
                return false;
            }

            _received[ interfaceName ].regularReceived = true;

            if ( _currentWaveId == otherWaveId ) {
                if ( !_allRegularReceived() ) {
                    return false;
                }

                // This node was elected.
                if ( _id == otherWaveId ) {
                    _leaderId = _id;
                    _resetReceived();
                    _received[ interfaceName ].regularReceived = true;
                    _messageType = MessageType::LEADER_MESSAGE;
                    _relayType = RelayType::SEND_TO_ALL;
                    _confChanges.push_back( { ConfigAction::ADD_IP, { "rl0", _leaderAddress, _mask } } );
                } else {
                    _messageType = MessageType::ELECTION_MESSAGE;
                    _relayType = RelayType::SEND_TO_PARENT;
                    _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                }
                return true;
            }

            // _currentWaveId > otherWaveId
            if ( _received.size() == 1 ) {
                if ( _restart ) {
                    _restart = false;
                }
            }
            _relayType = ( _received.size() == 1 ) ? RelayType::SEND_TO_PARENT : RelayType::SEND_TO_NEIGHBORS;
            _messageType = MessageType::ELECTION_MESSAGE;
            _resetReceived();
            _received[ interfaceName ].regularReceived = true;
            _parent = interfaceName;
            _currentWaveId = otherWaveId;
            _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
            return true;
        }

        bool _onLeaderMessage( const std::string& interfaceName, int otherWaveId ) {
            if ( _noLeaderReceived() ) {
                _received[ interfaceName ].leaderReceived = true;
                _leaderId = otherWaveId;
                _currentWaveId = _id;
                _parent = "rl0";
                _messageType = MessageType::LEADER_MESSAGE;
                _relayType = RelayType::SEND_TO_ALL;
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            }
            return false;
        }

        bool _onInitiateMessage( const std::string& interfaceName ) {
            _received[ interfaceName ].initiated = true;
            if ( _managedInterfaces.size() == 7 && _allInitiated() ) {
                _messageType = MessageType::ELECTION_MESSAGE;
                _relayType = RelayType::SEND_TO_NEIGHBORS;
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            }
            return false;
        }

    public:
        EchoElection( int id, const Ip6Addr& leaderAddr, uint8_t mask )
        : _id(id), _leaderAddress( leaderAddr ), _mask(mask) {
            _leaderId = -1;
            _currentWaveId = _id;
            _parent = "rl0";
            _restart = false;
            _messageType = MessageType::ELECTION_MESSAGE;
            _relayType = RelayType::SEND_TO_NEIGHBORS;
        }

        virtual bool onMessage( const std::string& interfaceName,
                                rofi::hal::PBuf packetWithHeader ) override {
            auto packet = PBuf::own( pbuf_free_header( packetWithHeader.release(), IP6_HLEN ) );
            MessageType type = as< MessageType >( packet.payload() );
            int waveId = as< int >( packet.payload() + sizeof( MessageType ) );
            _restart = as< bool >( packet.payload() + sizeof( MessageType ) + sizeof( int ) );

            switch ( type ) {
                case MessageType::ELECTION_MESSAGE:
                    return _onElectionMessage( interfaceName, waveId );
                case MessageType::LEADER_MESSAGE:
                    return _onLeaderMessage( interfaceName, waveId );
                default:
                    return _onInitiateMessage( interfaceName );
            }
            return false;

        }

        virtual bool afterMessage( const Interface& interface, 
                                   std::function< void ( PBuf&& ) > fun, void* /* args */ ) override {
            if ( _relayType == RelayType::DO_NOT_SEND ) {
                return false;
            }
            
            if ( _relayType == RelayType::SEND_TO_NEIGHBORS && interface.name() == _parent ) {
                return false;
            }

            if ( _relayType == RelayType::SEND_TO_PARENT && interface.name() != _parent ) {
                return false;
            }

            auto packet = PBuf::allocate( sizeof( MessageType ) + sizeof( int ) + sizeof( bool ) );
            as< MessageType >( packet.payload() ) = _messageType;
            as< int >( packet.payload() + sizeof( MessageType ) ) = ( _messageType == MessageType::LEADER_MESSAGE ) ? _leaderId : _currentWaveId;
            as< bool >( packet.payload() + sizeof( MessageType ) + sizeof( int ) ) = _restart;
            
            fun( std::move( packet ) );
            return false;
        }

        virtual bool hasConfigUpdates() const override { return!_confChanges.empty(); }

        virtual std::vector< std::pair< ConfigAction, ConfigChange > > getConfigUpdates() const {
            return _confChanges;
        }

        virtual void clearUpdates() { _confChanges.clear(); }

        virtual bool addInterface( const Interface& interface ) {
            if ( manages( interface ) ) {
                return false;
            }
            _managedInterfaces.push_back( std::reference_wrapper( interface ) );

            if ( const_cast< Interface& >( interface ).isConnected() ) {
                _received[ interface.name() ].leaderReceived = false;
            }

            _relayType = RelayType::DO_NOT_SEND;

            if ( _managedInterfaces.size() == 7 && _currentWaveId == _id ) {
                _messageType = INITIATE_MESSAGE;
                _relayType = SEND_TO_NEIGHBORS;
            }

            return true;
        }

        virtual bool removeInterface( const Interface& interface ) {
            auto it = std::find_if( _managedInterfaces.begin(), _managedInterfaces.end()
                                , [ &interface ]( const auto& i ) { return interface == i; } );
            if ( it == _managedInterfaces.end() )
                return false;

            std::swap( *it, _managedInterfaces.back() );
            _managedInterfaces.pop_back();
            return true;
        }

        virtual bool onInterfaceEvent( const Interface& interface, bool connected ) override {
            if ( _restart ) {
                return false;
            }

            _restart = true;

            if ( connected ) {
                _received[ interface.name() ].regularReceived = false;
            } else {
                _received.erase( interface.name() );
            }

            _messageType = MessageType::ELECTION_MESSAGE;
            _relayType = RelayType::SEND_TO_NEIGHBORS;
            _confChanges.push_back( { ConfigAction::RESPOND, { interface.name(), Ip6Addr( "::" ), 0 } } );
            return true;
        }

        virtual bool manages( const Interface& interface ) const override {
            return std::ranges::any_of( _managedInterfaces, [ &interface ]( const Interface& i ) {
                return interface == i;
            } );
        }

        virtual Ip6Addr address() const override { return Ip6Addr( "ff02::ea:ea" ); }

        virtual std::string name() const override { return "echo-election"; }

        virtual std::string info() const override {
            std::string str = Protocol::info();
            std::stringstream ss;
            ss << "; leader id: " << _leaderId << " leader address: " << _leaderAddress;
            return str  + ss.str();
        }
    };
}