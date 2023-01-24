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

enum MessageType {
    TOKEN_MESSAGE,
    LEADER_MESSAGE,
    RESET_REQUEST,
    RESET_RESPONSE
};

enum RelayType {
    SEND_TO_NEIGHBORS,
    SEND_TO_PARENT,
    RESPONSE_TO_PARENT, //ToDo: Think about changing this - used in reset.
    IGNORE_PARENT,
    DO_NOT_SEND,
};


namespace rofi::net {
    /**
     * An election protocol built on the principles of extinction and the echo wave protocol.
     * Restart here is done in a 'lazy' or 'sensitive' manner, restarting whenever an interface event occurs. This
     * should not change the outcome if the interface change did not affect the overall topology in any serious
     * manner ( e.g. the bot is split into two independent bots ), but does invoke more restarts than may be necessary.
     * However, it also saves on message complexity ( in comparison with simple-elect, we only send a simple message instead of the entire table )
    */
    class EchoElection : public Protocol {

        struct InterfaceReceived {
            bool regularReceived;
            bool leaderReceived;
            bool resetReceived;
        };

        std::vector< std::reference_wrapper< const Interface > > _managedInterfaces;
        std::set< std::string > _interfaceWithCb;

        std::vector< std::pair< ConfigAction, ConfigChange > > _confChanges;

        Ip6Addr _leaderAddress;
        uint8_t _mask;
        int _id;
        
        int _winnerId;
        int _currentWaveId;
        Interface::Name _parent;

        MessageType _messageType;
        RelayType _relayType;

        std::map< Interface::Name, InterfaceReceived > _received;


        void _resetReceived( bool includeReset ) {
            for ( auto& [ key, _ ] : _received ) {
                _received[ key ].regularReceived = false;
                _received[ key ].leaderReceived = false;
                if ( includeReset ) {
                    _received[ key ].resetReceived = false;
                }
            }
        }

        // ToDo: Merge with _checkReceived
        bool _checkReset( bool any ) {
            for ( auto [ _, value ] : _received ) {
                if ( any && value.resetReceived ) {
                    return true; 
                }

                if ( !any && !value.resetReceived ) {
                    return false;
                }
            }

            return !any;
        }

        bool _checkReceived( bool any, bool checkLeaderMessages ) {
            for ( auto& [ _, value ] : _received ) {
                bool checkResult = checkLeaderMessages ? value.leaderReceived : value.regularReceived;

                // Checking for at least one being true.
                if ( any && checkResult ) {
                    return true;
                }

                // Checking for all being true, so if we find one that is not, return false.
                if ( !any && !checkResult ) {
                    return false;
                }
            }
            
            return !any;
        }

        bool _election( const std::string& interfaceName, 
                                MessageType messageType, int waveId ) {
            bool result = false;
            if ( messageType == MessageType::LEADER_MESSAGE ) {
                // If no leader messages were received up until this point, send out leader messages to neighbours.
                if ( ! _checkReceived( true, true ) ) {
                    _messageType = MessageType::LEADER_MESSAGE;
                    _relayType = RelayType::SEND_TO_NEIGHBORS;
                    _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                    result = true;
                }

                _received[ interfaceName ].leaderReceived = true;

                // If this was a leader before, but is now about to be 'dethroned', we must remove the address from it.
                if ( waveId != _winnerId && _winnerId == _id ) {
                    _confChanges.push_back( { ConfigAction::REMOVE_IP, { "rl0", _leaderAddress, _mask } } );
                    result = true;
                }
                _winnerId = waveId;
                _currentWaveId = _winnerId;
            } else {
                _received[ interfaceName ].regularReceived = true;

                if ( waveId < _currentWaveId ) {
                    _resetReceived( true );
                    _currentWaveId = waveId;
                    _parent = interfaceName;

                    _messageType = TOKEN_MESSAGE;
                    _relayType = IGNORE_PARENT;

                    _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                    result = true;
                } else if ( waveId == _currentWaveId ) {
                    // Did all active interfaces receive a regular type message?
                    if ( ! _checkReceived( false, false ) ) {
                        return result;
                    }

                    if ( _currentWaveId == _id ) {
                        _messageType = MessageType::TOKEN_MESSAGE;
                        _relayType = RelayType::SEND_TO_NEIGHBORS;
                    } else {
                        _messageType = MessageType::LEADER_MESSAGE;
                        _relayType = RelayType::SEND_TO_PARENT;
                    }
                    _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                    result = true;
                }
            }

            if ( _checkReceived( false, true ) && _winnerId == _id ) {
                _confChanges.push_back( { ConfigAction::ADD_IP, { "rl0", _leaderAddress, _mask } } );
                _relayType = RelayType::DO_NOT_SEND;
                return true;
            }

            return result;
        }

        bool _reset( const std::string& interfaceName, MessageType messageType, int waveId ) {
            bool result = false;
            
            if ( _messageType != MessageType::RESET_REQUEST && _messageType != MessageType::RESET_RESPONSE ) {
                _parent = interfaceName;
            } else if ( _relayType != RelayType::IGNORE_PARENT ) {
                _parent = "rl0";
            }

            _received[ interfaceName ].resetReceived = true;
            
            if ( messageType == MessageType::RESET_REQUEST ) {

                // If all interfaces received a reset message -> notify parent with RESET_RESPONSE
                if ( _checkReset( false ) ) { 
                    _currentWaveId = _id;
                    _messageType = MessageType::RESET_RESPONSE;
                    _relayType = RelayType::RESPONSE_TO_PARENT;
                } else {
                    _messageType = MessageType::RESET_REQUEST;
                    _relayType = RelayType::IGNORE_PARENT;
                }
                result = true;
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
            }

            // We treat RESET_RESPONSE partially as a token message.
            else {
                // All recesponses received
                if ( _checkReset( false ) ) {
                    //perform reset
                    _resetReceived( false );
                    _currentWaveId = _id;
                    _parent = "rl0";
                    _messageType = MessageType::RESET_RESPONSE;
                    
                    // This will send RESET_RESPONSE back to parent but TOKEN_MESSAGE to neighbours.
                    _relayType = RelayType::RESPONSE_TO_PARENT;
                    result = true;
                    _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                }

                // Relay the received wave.
                if ( waveId < _currentWaveId) {
                    _currentWaveId = waveId;
                    _parent = interfaceName;
                    result = true;
                    _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                }
            }
            return result;
        }

    public: 
        EchoElection( int id, const Ip6Addr& leaderAddr, uint8_t mask ) 
        : _leaderAddress( leaderAddr ), _mask( mask ), _id( id ) {
            _winnerId = -1;
            _currentWaveId = _id;
            _parent = "rl0";
            _messageType = MessageType::TOKEN_MESSAGE;
            _relayType = RelayType::SEND_TO_NEIGHBORS;
        }

        virtual bool onMessage( const std::string& interfaceName,
                                rofi::hal::PBuf packetWithHeader ) override {
            bool result = false;
            
            auto packet = PBuf::own( pbuf_free_header( packetWithHeader.release(), IP6_HLEN ) );
            MessageType messageType = as< MessageType >( packet.payload() );
            int waveId = as< int >( packet.payload() + sizeof( MessageType ) );

            if ( messageType == MessageType::LEADER_MESSAGE || messageType == MessageType::TOKEN_MESSAGE ) {
                return _election( interfaceName, messageType, waveId );
            }
            return _reset( interfaceName, messageType, waveId );
        }

        virtual bool afterMessage( const Interface& interface,
                                   std::function< void ( PBuf&& ) > fun, void* /* args */ ) override {
            // ToDo: Think about making this nicer.
            if ( _relayType == RelayType::DO_NOT_SEND ) {
                return false;
            }

            if ( _relayType == IGNORE_PARENT && interface.name() == _parent ) {
                return false;
            }

            if ( _relayType == SEND_TO_PARENT && interface.name() != _parent ) {
                return false;
            }

            if ( _relayType == RESPONSE_TO_PARENT && interface.name() == _parent ) {
                _messageType = MessageType::RESET_RESPONSE;
            } else if ( _relayType == RESPONSE_TO_PARENT ) {
                _messageType = MessageType::TOKEN_MESSAGE;
            }

            auto packet = PBuf::allocate( sizeof( MessageType ) + sizeof( int ) );
            as< MessageType >( packet.payload() ) = _messageType;
            as< int >( packet.payload() + sizeof( MessageType ) ) = _currentWaveId;

            fun( std::move( packet ) );
            
            // Ensures the reset is completed and every bot moves on to the ELECTION process.
            if ( _messageType == MessageType::RESET_RESPONSE && interface.name() == "rd6" ) { // TODO: FIX THIS
                _messageType = MessageType::TOKEN_MESSAGE;
            }
            return false;
        }

        virtual bool hasConfigUpdates() const override { return!_confChanges.empty(); }

        virtual std::vector< std::pair< ConfigAction, ConfigChange > > getConfigUpdates() const {
            return _confChanges;
        }

        virtual void clearUpdates() { _confChanges.clear(); }

        virtual bool onInterfaceEvent( const Interface& interface, bool connected ) override { 
            _parent = "rl0";
            _resetReceived( true );
            _currentWaveId = _id;

            if ( _messageType != MessageType::RESET_REQUEST && _messageType != MessageType::RESET_RESPONSE ) {
                _confChanges.push_back( { ConfigAction::RESPOND, { interface.name(), Ip6Addr( "::" ), 0 } } );
            }

            _messageType = MessageType::RESET_REQUEST;
            _relayType = RelayType::SEND_TO_NEIGHBORS;

            if ( !connected ) {
                _received.erase( interface.name() );
            }

            return true; 
        }

        virtual bool addInterface( const Interface& interface ) {
            if ( manages( interface ) ) {
                return false;
            }

            _managedInterfaces.push_back( std::reference_wrapper( interface ) );
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
            ss << "; leader id: " << _winnerId << " leader address: " << _leaderAddress;
            return str  + ss.str();
        }
    };
}