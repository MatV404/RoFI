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
    class TraverseElection : public Protocol {
        enum TokenType {
            INITIATE,
            ANNEXING,
            CHASING,
            LEADER,
            NONE,
        };

        struct Token {
            TokenType type;
            int phase;
            int identity;
            int hopCount;
        };

        struct ChannelInfo {
            bool initiated;
            bool sent;
            bool leaderReceived;
        };

        std::vector< std::reference_wrapper< const Interface > > _managedInterfaces;
        std::set< std::string > _interfaceWithCb;
        std::vector< std::pair< ConfigAction, ConfigChange > > _confChanges;

        std::map< Interface::Name, ChannelInfo > _channels;
        Interface::Name _parent = "rl0";
        Interface::Name _next;

        const Ip6Addr&  _leaderAddress;
        uint8_t _mask;
        int _leaderId;
        int _id;

        int _currentPhase;
        int _chasedPhase;
        
        int _maxHops;
        
        int _tokenId;
        int _waitingTokenId;

        TokenType _sendType;
        std::function< void ( bool, ElectionStatus ) >& _electionChangeCallback;

        bool _allInitiated( ) {
            for ( const auto& [ _ , value ] : _channels ) {
                if ( !value.initiated ) {
                    return false;
                }
            }
            return true;
        }

        void _determineNextNode() {
            for ( const auto& [ key, _ ] : _channels ) {
                if ( key != _parent && !_channels[ key ].sent ) {
                    _next = key;
                    return;
                }
            }

            if ( _parent != "rl0" && !_channels[ _parent ].sent ) {
                _next = _parent;
                return;
            }

            // The traversal algorithm ends.
            _next = "rl0";
        }

        void _resetSent() {
            for ( const auto& [ key, _ ] : _channels ) {
                _channels[ key ].sent = false;
            }
        }

        void _resetTraversal( const std::string& interfaceName, int tokenId, int phase, int hops ) {
            _resetSent();
            _tokenId = tokenId;
            _currentPhase = phase;
            _parent = interfaceName;
            _chasedPhase = -1;
            _maxHops = hops;
            _waitingTokenId = -1;
        }

        bool _leaderActions( int id, const std::string& interfaceName ) {
            _channels[ interfaceName ].leaderReceived = true;

            if ( id == _id ) {
                _confChanges.push_back( { ConfigAction::ADD_IP, { "rl0", _leaderAddress, _mask } } );
                _electionChangeCallback( true, ElectionStatus::LEADER );
            } else {
                _confChanges.push_back( { ConfigAction::REMOVE_IP, { "rl0", _leaderAddress, _mask } } );
                _electionChangeCallback( true, ElectionStatus::FOLLOWER );
            }

            _sendType = TokenType::LEADER;
            _leaderId = id;
            _resetTraversal( "rl0", -1, -1, -1 );
            return true;
        }

        bool _onAnnexingToken( const std::string& interfaceName,
                               Token receivedToken ) {
            if ( _parent == "rl0" && receivedToken.identity != _id ) {
                _parent = interfaceName;
                for ( const auto& [ key, _ ] : _channels ) {
                    _channels[ key ].leaderReceived = false;
                }
                _electionChangeCallback( false, ElectionStatus::UNDECIDED );
            }

            // Tokens of lower phases are redundant and thus killed off.
            if ( receivedToken.phase < _currentPhase ) {
                return false;
            }

            // Sets up the conditions for leader token relay after restart.
            if ( _sendType == TokenType::LEADER ) {
                for ( const auto& [ key, _ ] : _channels ) {
                    _channels[ key ].leaderReceived = false;
                }
            }

            if ( _maxHops < receivedToken.hopCount ) {
                _maxHops = receivedToken.hopCount;
            }

            // A token of larger phase appears, the node has to reset to respect new traversal.
            if ( receivedToken.phase > _currentPhase ) {
                _resetTraversal( interfaceName, receivedToken.identity, receivedToken.phase, receivedToken.hopCount );
                _sendType = TokenType::ANNEXING;
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                _determineNextNode(); // Since this is when a 're-initialization' occurs, we should never receive 'rl0' as next here.
                return true;
            }

            // A token is waiting here, met by another token. We combine them to a higher phase token.
            if ( _waitingTokenId != -1 ) {
                _resetTraversal( "rl0", _id, _currentPhase + 1, -1 );
                _sendType = TokenType::ANNEXING;
                _determineNextNode();
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            }

            // The node is annexed, any lower phase waiting tokens are deleted.
            if ( receivedToken.identity == _tokenId 
                 && _chasedPhase != receivedToken.phase ) {
                _waitingTokenId = -1;
                _determineNextNode();
                if ( _next == "rl0" ) {
                    return _leaderActions( _id, interfaceName );
                }
                _sendType = TokenType::ANNEXING;
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            }

            if ( _chasedPhase == receivedToken.phase 
                || _tokenId > receivedToken.identity ) {
                _waitingTokenId = receivedToken.identity;
                return false;
            }

            _chasedPhase = _currentPhase;
            _sendType = TokenType::CHASING;
            _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
            return true;
        }

        bool _onChasingToken( const std::string& interfaceName,
                              Token receivedToken ) {
            if ( receivedToken.phase < _currentPhase ) {
                return false;
            }

            if ( receivedToken.phase == _currentPhase 
                 && receivedToken.identity == _tokenId  
                 && _maxHops > receivedToken.hopCount 
                 && _chasedPhase != receivedToken.phase && _waitingTokenId == -1 ) {
                _chasedPhase = receivedToken.phase;
                _sendType = TokenType::CHASING;
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            } 

            // The chasing token catches up, the two tokens merge.
            if ( receivedToken.phase == _currentPhase && _waitingTokenId != -1 ) {
                _resetTraversal( "rl0", _id, _currentPhase + 1, -1 );
                _sendType = TokenType::ANNEXING;
                _resetSent();
                _determineNextNode();
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            }

            _waitingTokenId = receivedToken.identity;
            return false;
        }

        bool _onInitiateToken( const std::string& interfaceName ) {
            _channels[ interfaceName ].initiated = true;
            if ( _managedInterfaces.size() == 7 && _allInitiated() && _currentPhase == -1 ) {
                _sendType = TokenType::ANNEXING;
                _determineNextNode();
                _currentPhase = 0;
                _tokenId = _id;
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            }
            return false;
        }

        bool _onLeaderToken( const std::string& interfaceName,
                             int waveId ) {
            for ( const auto& [ _ , recv ] : _channels ) {
                if ( recv.leaderReceived ) {
                    return false;
                }
            }

            return _leaderActions( waveId, interfaceName );
        }

    public:
        TraverseElection( int id, const Ip6Addr& leaderAddress, uint8_t mask, std::function< void( bool, ElectionStatus ) > cb )
        : _leaderAddress( leaderAddress ), _mask( mask ), _id( id ), _electionChangeCallback( cb ) {
            _leaderId = -1;
            _tokenId = -1;
            _maxHops = -1;
            _currentPhase = -1;
            _chasedPhase = -1;
            _waitingTokenId = -1;
        }

        virtual ~TraverseElection() = default;

        virtual bool onMessage( const std::string& interfaceName,
                                rofi::hal::PBuf packetWithHeader ) override {
            auto packet = PBuf::own( pbuf_free_header( packetWithHeader.release(), IP6_HLEN ) );
            Token receivedToken = as< Token >( packet.payload() );
            switch ( receivedToken.type ) {
                case TokenType::ANNEXING:
                    return _onAnnexingToken( interfaceName, receivedToken );
                case TokenType::CHASING:
                    return _onChasingToken( interfaceName, receivedToken );
                case TokenType::INITIATE:
                    return _onInitiateToken( interfaceName  );
                case TokenType::LEADER:
                    return _onLeaderToken( interfaceName, receivedToken.identity );
                default:
                    assert( false && "Invalid message type receved\n" );
            }

            return false;
        }

        virtual bool afterMessage( const Interface& interface, 
                                   std::function< void ( PBuf&& ) > fun, void* /* args */ ) override {
            if ( _sendType == TokenType::NONE ) {
                return false;
            }

            if ( _sendType == TokenType::ANNEXING) {
                if (  interface.name() != _next ) {
                    return false;
                }
                _channels[ interface.name() ].sent = true;
                _maxHops++;
            }

            if ( _sendType == TokenType::CHASING && interface.name() != _next ) {
                return false;
            }

            Token sendToken;
            sendToken.hopCount = _maxHops;
            sendToken.identity = ( _sendType == TokenType::LEADER ) ? _leaderId : _tokenId;
            sendToken.phase = _currentPhase;
            sendToken.type = _sendType;

            auto packet = PBuf::allocate( sizeof( Token ) );
            as< Token >( packet.payload() ) = sendToken;

            fun( std::move( packet ) );

            return false;
        }

        virtual bool hasConfigUpdates() const override { return!_confChanges.empty(); }

        virtual std::vector< std::pair< ConfigAction, ConfigChange > > getConfigUpdates() const {
            return _confChanges;
        }

        virtual void clearUpdates() { _confChanges.clear(); }

        virtual bool onInterfaceEvent( const Interface& interface, bool connected ) override {
            if ( connected ) {
                _channels[ interface.name() ].sent = false;
            } else {
                if ( _channels.find( interface.name() ) == _channels.end() ) {
                    return false;
                }
                _channels.erase( interface.name() );
            }

            for ( const auto& [ key, _ ] : _channels ) {
                _channels[ key ].leaderReceived = false;
            }
            _determineNextNode();
            if ( _next == "rl0" ) {
                return _leaderActions( _id, interface.name() );
            }
            _sendType = TokenType::ANNEXING;
            _tokenId = _id;
            _currentPhase = 0;
            _confChanges.push_back( { ConfigAction::RESPOND, { interface.name(), Ip6Addr( "::" ), 0 } } );

            return true;
        }

        virtual bool addInterface( const Interface& interface ) {
            if ( manages( interface ) ) {
                return false;
            }

            _managedInterfaces.push_back( std::reference_wrapper( interface ) );

            if ( const_cast< Interface& >(interface).isConnected() ) {
                _channels[ interface.name() ].sent = false;
            }
           
            _sendType = TokenType::NONE;

            if ( _managedInterfaces.size() == 7 && _currentPhase == -1 ) {
                _sendType = TokenType::INITIATE;
                return true;
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

        virtual bool manages( const Interface& interface ) const override {
            return std::ranges::any_of( _managedInterfaces, [ &interface ]( const Interface& i ) {
                return interface == i;
            } );
        }

        virtual Ip6Addr address() const override { return Ip6Addr( "ff02::ea:ea" ); }

        virtual std::string name() const override { return "traversal-election"; }

        virtual std::string info() const override {
            std::string str = Protocol::info();
            std::stringstream ss;
            ss << "; leader id: " << _leaderId << " leader address: " << _leaderAddress;
            return str  + ss.str();
        }
    };
}