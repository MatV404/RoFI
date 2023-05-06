#pragma once

#include "lwip++.hpp"
#include <networking/protocol.hpp>
#include <networking/interface.hpp>
#include <networking/routingTable.hpp>

#include <enums.hpp>
#include <atoms/util.hpp>

#include <vector>
#include <map>
#include <set>
#include <utility>


namespace rofi::net {
    using namespace rofi::leadership;
    class EchoElection : public Protocol {
        struct ConnectionInfo {
            bool receivedElection = false;
            bool receivedLeader = false;
            bool initiateReceived = false;
        };

        std::vector< std::reference_wrapper< const Interface > > _managedInterfaces;
        std::set< std::string > _interfaceWithCb;

        std::vector< std::pair< ConfigAction, ConfigChange > > _confChanges;

        Ip6Addr _id;
        Ip6Addr _leaderId;
        Ip6Addr _currentWaveId;

        Interface::Name _parent = "rl0";
        Interface::Name _connected = "rl0";

        MessageType _messageType;

        std::map< Interface::Name, ConnectionInfo > _connections;
        std::function< void ( Ip6Addr, ElectionStatus ) >& _electionChangeCallback;

        void _resetReceived() {
            for ( const auto& [ key, _ ] : _connections ) {
                _connections[ key ].receivedLeader = false;
                _connections[ key ].receivedElection = false;
            }
        }

        bool _allElectionReceived() {
            for ( const auto& [ key, _ ] : _connections ) {
                if ( !_connections[ key ].receivedElection ) {
                    return false;
                }
            }
            return true;
        }

        bool _allInitiated() {
            for ( const auto& [ key, _ ] : _connections ) {
                if ( !_connections[ key ].initiateReceived ) {
                    return false;
                }
            }
            return true;
        }

        bool _leaderReceived( bool all ) {
            for ( const auto& [ key, _ ] : _connections ) {
                if ( !all && _connections[ key ].receivedLeader ) {
                    return false;
                }
                if ( all && !_connections[ key ].receivedLeader ) {
                    return false;
                }
            }
            return true;
        }

        bool _onElectionMessage( const std::string& interfaceName, Ip6Addr waveId ) {
            if ( waveId > _currentWaveId ) {
                // Restart occured.
                if ( _currentWaveId == _id ) {
                    _electionChangeCallback( _id, ElectionStatus::UNDECIDED );
                    _parent = "rl0";
                    _resetReceived();
                    _messageType = MessageType::ELECTION_MESSAGE;
                    _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
                    return true;
                }
                return false;
            }

            _connections[ interfaceName ].receivedElection = true;

            if ( waveId == _currentWaveId ) {
                if ( !_allElectionReceived() ) {
                    return false;
                }

                // This means that the node was elected.
                if ( waveId == _id ) {
                    _electionChangeCallback( _id, ElectionStatus::LEADER );
                    _leaderId = _id;
                    _parent = "rl0";
                    _connections[ interfaceName ].receivedLeader = true;
                    _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
                    _messageType = MessageType::LEADER_MESSAGE;
                    return true;
                }

                _messageType = MessageType::ELECTION_MESSAGE;
                _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
                return true;
            }

            // waveId < _currentWaveId, restart.
            if ( _messageType == MessageType::LEADER_MESSAGE ) {
                _electionChangeCallback( _leaderId, ElectionStatus::UNDECIDED );
            }
            _currentWaveId = waveId;
            _parent = interfaceName;
            _resetReceived();
            _connections[ interfaceName ].receivedElection = true;

            _messageType = MessageType::ELECTION_MESSAGE;
            _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
            return true;
        }

        bool _onLeaderMessage( const std::string& interfaceName, Ip6Addr waveId ) {
            if ( ( _leaderReceived( false ) && waveId != _id ) || waveId < _leaderId ) {
                _electionChangeCallback( waveId, ElectionStatus::FOLLOWER );
                _connections[ interfaceName ].receivedLeader = true;
                _leaderId = waveId;
                _currentWaveId = _id;
                _parent = interfaceName;
                _messageType = MessageType::LEADER_MESSAGE;
                _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
                return true;
            }

            return false;
        }

        bool _onInitiateMessage( const std::string& interfaceName ) {
            _connections[ interfaceName ].initiateReceived = true;

            if ( _managedInterfaces.size() == 7 && _allInitiated() ) {
                _messageType = MessageType::ELECTION_MESSAGE;
                _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
                return true;
            }

            return false;
        }

        bool _onConnectMessage( const std::string& interfaceName, Ip6Addr otherLeader ) {
            if ( otherLeader < _leaderId ) {
                return _onLeaderMessage( interfaceName, otherLeader );
            }

            if ( otherLeader == _leaderId ) {
                return false;
            }

            if ( _id == _leaderId ) {
                _electionChangeCallback( _leaderId, ElectionStatus::CHANGED_FOLLOWERS );
                return false;
            }

            _messageType = MessageType::FOLLOWER_CHANGE_MESSAGE;
            _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
            return true;
        }

        bool _onConnectionEstablishedMessage() {
            if ( _leaderId == _id ) {
                _electionChangeCallback( _leaderId, ElectionStatus::CHANGED_FOLLOWERS );
                return false;
            }

            _messageType = MessageType::FOLLOWER_CHANGE_MESSAGE;
            _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
            return true;
        }

        void _printMessage( bool received, const std::string& interfaceName,
                            MessageType type, Ip6Addr waveId ) {
            if ( received ) {
                std::cout << "Received on " << interfaceName;
            } else {
                std::cout << "Sending to " << interfaceName; 
            }

            if ( MessageType::ELECTION_MESSAGE == type ) {
                std::cout << " Election Message ";
            } 
            else if ( MessageType::LEADER_MESSAGE == type ) {
                std::cout << " Leader Message ";
            } else if ( MessageType::INITIATE_MESSAGE == type ) {
                std::cout << " Initiate Message ";
            } else if ( MessageType::CONNECT_MESSAGE == type ) {
                std::cout << " Connection Message ";
            } else {
                std::cout << " Followers Changed Message ";
            }

            std::cout << "with ID " << waveId << "\n";
        }

    public:
        EchoElection( const Ip6Addr& addr, std::function< void( Ip6Addr, ElectionStatus ) > cb )
        : _id( addr ), _leaderId( addr ), _currentWaveId( addr ), _electionChangeCallback( cb ) {
            _parent = "rl0";
            _messageType = MessageType::INITIATE_MESSAGE;
        }

        virtual ~EchoElection() = default;

        virtual bool onMessage( const std::string& interfaceName,
                               rofi::hal::PBuf packetWithHeader ) override {
            auto packet = PBuf::own( pbuf_free_header( packetWithHeader.release(), IP6_HLEN ) );
            
            MessageType type = as< MessageType >( packet.payload() );
            Ip6Addr waveId = as< Ip6Addr >( packet.payload() + sizeof( MessageType ) );

            // _printMessage( true, interfaceName, type, waveId );

            switch ( type ) {
                case MessageType::LEADER_MESSAGE:
                    return _onLeaderMessage( interfaceName, waveId );
                case MessageType::ELECTION_MESSAGE:
                    return _onElectionMessage( interfaceName, waveId );
                case MessageType::INITIATE_MESSAGE:
                    return _onInitiateMessage( interfaceName );
                case MessageType::CONNECT_MESSAGE:
                    return _onConnectMessage( interfaceName, waveId );
                case MessageType::FOLLOWER_CHANGE_MESSAGE:
                    return _onConnectionEstablishedMessage();
            }

            std::cout << "Unexpected Behavior.\n";
            return false;
        }

        virtual bool afterMessage( const Interface& interface, 
                                   std::function< void ( PBuf&& ) > fun, void* /* args */ ) override {
            if ( _managedInterfaces.size() != 7 || ! const_cast< Interface& >( interface ).isConnected() ) {
                return false;
            }
            
            if ( _messageType == MessageType::ELECTION_MESSAGE ) {
                if (  _allElectionReceived() && interface.name() != _parent ) {
                    return false;
                }

                if ( !_allElectionReceived() && interface.name() == _parent ) {
                    return false;
                }
            }

            if ( _messageType == MessageType::CONNECT_MESSAGE && interface.name() != _connected ) {
                return false;
            }

            if ( _messageType == MessageType::FOLLOWER_CHANGE_MESSAGE && interface.name() != _parent ) {
                return false;
            }

            

            PBuf packet = PBuf::allocate( sizeof( MessageType ) + Ip6Addr::size() );
            as< MessageType >( packet.payload() ) = _messageType;
            if ( _messageType == MessageType::ELECTION_MESSAGE || _messageType == MessageType::INITIATE_MESSAGE ) {
                as< Ip6Addr >( packet.payload() + sizeof( MessageType ) ) = _currentWaveId;
                // _printMessage( false, interface.name(), _messageType, _currentWaveId );
            } else {
                as< Ip6Addr >( packet.payload() + sizeof( MessageType ) ) = _leaderId;
                // _printMessage( false, interface.name(), _messageType, _leaderId );
            }
            

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
            if ( const_cast< Interface& >( interface ).isConnected() && _connections.find( interface.name() ) == _connections.end() ) {
                _connections[ interface.name() ].receivedElection = false;
            }
            return false;
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
            if ( !connected ) {
                // For some reason, the disconnect interface event is triggered twice. This is a safeguard against 
                // restarting the same process twice.
                if ( _connections.find( interface.name() ) == _connections.end() ) {
                    return false;
                }

                _connections.erase( interface.name() );
                if ( interface.name() == _parent ) {
                    _parent = "rl0";
                    _resetReceived();
                    _electionChangeCallback( _id, ElectionStatus::UNDECIDED );
                    if ( _connections.size() == 0 ) {
                        return _onElectionMessage( interface.name(), _id );
                    }
                    _messageType = MessageType::ELECTION_MESSAGE;
                    _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
                    return true;
                }

                if ( _id == _leaderId ) {
                    _electionChangeCallback( _leaderId, ElectionStatus::CHANGED_FOLLOWERS );
                    return false;
                }

                _messageType = MessageType::FOLLOWER_CHANGE_MESSAGE;
                _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
                return true;
            }

            if ( connected ) {
                _connections[ interface.name() ].receivedElection = true;
                _messageType = MessageType::CONNECT_MESSAGE;
                _connected = interface.name();
                _confChanges.push_back( { ConfigAction::RESPOND, { "", Ip6Addr( "::" ), 0 } } );
                return true;
            }

            return false;
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
            ss << "; leader id: " << _leaderId;
            return str  + ss.str();
        }
    };
}