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
    /**
     * An election protocol based on the election algorithm construction created by E. Korach,
     * S. Kutten and S. Moran, using a traversal algorithm to elect the node at which the network
     * traversal terminates.
     * Things to consider: The current way this algorithm is restarted, as it does so only at 
     * the connecting / disconnecting nodes, which may mean that the leader will be one of the
     * two nodes and a full election may not take place.
     *                     There is also a significant portion of code duplication, so it should
     * be considered whether it should be removed, or kept in in favour of code that is read more
     * easily due to the way this construction is structured. ( Currently, most of the code duplication 
     * happens due to the decoupling of 'Annexation' and 'Chasing' modes and their handling, as it is just
     * easier to understand what takes place that way. )
    */
    class TraverseElection : public Protocol {
        enum TokenMode {
            ANNEXING,
            CHASING,
            DISCOVERY,
            LEADER,
        };

        // ToDo: Consider if this is needed - is it really necessary to tie all
        // the data into a struct?
        struct Token {
            TokenMode mode;
            int phase;
            int identity;
            int hopCount;
        };

        std::vector< std::reference_wrapper< const Interface > > _managedInterfaces;
        std::set< std::string > _interfaceWithCb;
        std::vector< std::pair< ConfigAction, ConfigChange > > _confChanges;

        // Guard against double initialization after interface event.
        bool _restarted = false;

        const Ip6Addr&  _leaderAddress;
        uint8_t _mask;
        int _leaderId;

        int _phase;
        int _id;
        int _tokenId;
        int _maxHops;
        int _waitingTokenId = -1;
        int _chasedAtPhase = -1;
        TokenMode _tokenType;

        Interface::Name _nextNode;
        Interface::Name _parentNode = "rl0";
        Interface::Name _leaderFrom = "rl0";

        std::map< Interface::Name, bool > _sent;

        void _printToken( Token token ) {
            std::cout << "Token of ID " << token.identity << " with hop count of " << token.hopCount << " and phase " << token.phase << " with mode ";
            switch ( token.mode ) {
                case TokenMode::ANNEXING:
                    std::cout << " annexing.\n";
                    return;
                case TokenMode::CHASING:
                    std::cout << " chasing.\n";
                    return;
                case TokenMode::DISCOVERY:
                    std::cout << " discovery.\n";
                    return;
                default:
                    std::cout << " leader.\n";
                    return;
            }
        }

        void _determineNextNode() {
            for ( const auto& [ key, _ ] : _sent ) {
                if ( key != _parentNode && !_sent[ key ] ) {
                    _nextNode = key;
                    return;
                }
            }

            if ( _parentNode != "rl0" && !_sent[ _parentNode ] ) {
                _nextNode = _parentNode;
                return;
            }

            _nextNode = "rl0";
        }

        void _resetSent() {
            for ( const auto& [ key, _ ] : _sent ) {
                _sent[ key ] = false;
            }
        }

        bool _initialize( const std::string& interfaceName ) {
            _maxHops = -1;
            _phase = 0;
            _tokenId = _id;
            _tokenType = TokenMode::ANNEXING;
            _parentNode = "rl0";
            _determineNextNode();
            _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
            return true;
        }

        bool _leaderActions( int id, const std::string& interfaceName ) {
            if ( id == _id ) {
                _confChanges.push_back( { ConfigAction::ADD_IP, { "rl0", _leaderAddress, _mask } } );
            } else {
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
            }
            _tokenType = TokenMode::LEADER;
            _leaderId = id;
            _tokenId = -1;
            _maxHops = -1;
            _chasedAtPhase = -1;
            _phase = -1;
            _parentNode = "rl0";
            _resetSent();
            _restarted = false;
            return true;
        }

        // ToDo: Strange behaviour when two modules disconnect and then reconnect in a circle
        // topology. Check if it works as intended or not.
        bool _onAnnexingToken( const std::string& interfaceName, Token received ) {
            if ( _parentNode == "rl0" && received.identity != _id ) {
                _parentNode = interfaceName;
            }

            if ( received.phase < _phase ) {
                return false;
            }

            if ( _maxHops < received.hopCount ) {
                _maxHops = received.hopCount;
            }

            // The node is annexed, any lower phase waiting tokens are deleted.
            if ( received.phase > _phase  || ( received.phase == _phase // Might be removable. Since if phase > _phase and phase can't be < _phase then it can only be equal
                 && received.identity == _tokenId && _chasedAtPhase != received.phase )) {
                _phase = received.phase;
                _tokenId = received.identity;
                _waitingTokenId = -1;
                _determineNextNode();
                if ( _nextNode == "rl0" ) {
                    return _leaderActions( _id, interfaceName );
                }
                _tokenType = TokenMode::ANNEXING;
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            }

            // A token is waiting here, met by another token. We combine them to a higher phase token.
            if ( received.phase == _phase && _waitingTokenId != -1 ) { // again, the first comparison can be removed.
                _phase++;
                _tokenId = _id;
                _waitingTokenId = -1;
                _tokenType = TokenMode::ANNEXING;
                _determineNextNode();
                std::cout << _nextNode << "\n";
                if ( _nextNode == "rl0" ) {
                    return _leaderActions( _id, interfaceName );
                }
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            }

            if ( received.phase == _phase && ( _chasedAtPhase == received.phase || _tokenId > received.identity ) ) {
                _waitingTokenId = received.identity;
                return false;
            }

            _chasedAtPhase = _phase;
            _tokenType = TokenMode::CHASING;
            _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
            return true;
        }

        bool _onChasingToken( const std::string& interfaceName, Token received ) {
            if ( received.phase == _phase && received.identity == _tokenId  && _maxHops > received.hopCount 
                 && _chasedAtPhase != received.phase && _waitingTokenId == -1 ) {
                _chasedAtPhase = received.phase;
                _tokenType = TokenMode::CHASING;
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            } 
            
            if ( received.phase < _phase ) {
                return false;
            }

            if ( received.phase == _phase && _waitingTokenId != -1 ) {
                _phase++;
                _tokenType = TokenMode::ANNEXING;
                _tokenId = _id;
                _waitingTokenId = -1;
                _determineNextNode();
                if ( _nextNode == "rl0" ) {
                    return _leaderActions( _id, interfaceName );
                }
                _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                return true;
            }

            _waitingTokenId = received.identity;
            return false;
        }

    public:
        TraverseElection( int id, const Ip6Addr& leaderAddress, uint8_t mask )
        : _leaderAddress( leaderAddress ), _mask( mask ), _id( id ) {
            _phase = -1;
            _tokenId = -1;
            _maxHops = -1;
            _tokenType = TokenMode::DISCOVERY;
        }

        virtual bool onMessage( const std::string& interfaceName,
                                rofi::hal::PBuf packetWithHeader ) override {
            auto packet = PBuf::own( pbuf_free_header( packetWithHeader.release(), IP6_HLEN ) );
            Token receivedToken = as< Token >( packet.payload() );

            if ( receivedToken.mode != TokenMode::DISCOVERY ) {
                std::cout << "Token received on " << interfaceName << ": ";
                _printToken( receivedToken );
                std::cout << "Parent currently is: " << _parentNode << "\n";
            }
            // This is done in initialization to ensure we only care about interfaces that
            // have active connections. This also helps ignore the pad module.
            if ( receivedToken.mode == TokenMode::DISCOVERY ) {
                if ( _sent.find( interfaceName ) == _sent.end() ) {
                    _sent[ interfaceName ] = false;
                    _nextNode = interfaceName;
                    _tokenType = TokenMode::DISCOVERY;
                    _confChanges.push_back( { ConfigAction::RESPOND, { interfaceName, Ip6Addr( "::" ), 0 } } );
                    return true;
                }

                // If we already received a Discovery Token on this interface and we've not yet started
                // traversal, start it.
                if ( _tokenType != TokenMode::ANNEXING && _tokenId == -1 ) {
                    return _initialize( interfaceName );
                }

                // Otherwise, the received Discovery token is just ignored.
                return false;
            }

            if ( receivedToken.mode == TokenMode::LEADER && _tokenType != TokenMode::LEADER ) {
                if ( _id != receivedToken.identity ) {
                    _confChanges.push_back( { ConfigAction::REMOVE_IP, { "rl0", _leaderAddress, _mask } } );
                }
                return _leaderActions( receivedToken.identity, interfaceName );
            }

            if ( receivedToken.mode == TokenMode::ANNEXING ) {
                return _onAnnexingToken( interfaceName, receivedToken );
            }

            if ( receivedToken.mode == TokenMode::CHASING ) {
                return _onChasingToken( interfaceName, receivedToken );
            }

            return false;
        }

        virtual bool afterMessage( const Interface& interface,
                                   std::function< void ( PBuf&& ) > fun, void* /* args */ ) override {
            if ( _tokenType != TokenMode::LEADER && interface.name() != _nextNode ) {
                return false;
            }
            
            if ( _tokenType == TokenMode::LEADER && interface.name() == _leaderFrom ) {
                return false;
            }

            // We don't need to update maxHops with any other token type, 
            // since the chasing token only stores what it saw in the node.
            if ( _tokenType == TokenMode::ANNEXING ) {
                _sent[ interface.name() ] = true;
                _maxHops++;
            }

            Token toSend;

            toSend.hopCount = _maxHops;
            toSend.identity = ( _tokenType == TokenMode::LEADER ) ? _leaderId : _tokenId;
            toSend.mode = _tokenType;
            toSend.phase = _phase;

            auto packet = PBuf::allocate( sizeof( Token ) );
            as< Token >( packet.payload() ) = toSend;
            fun( std::move( packet ) );

            return false;
        }

        virtual bool hasConfigUpdates() const override { return!_confChanges.empty(); }

        virtual std::vector< std::pair< ConfigAction, ConfigChange > > getConfigUpdates() const {
            return _confChanges;
        }

        virtual void clearUpdates() { _confChanges.clear(); }

        virtual bool onInterfaceEvent( const Interface& interface, bool connected ) override {
            // A workaround to disconnect being triggered twice.
            if ( _restarted ) {
                return false;
            }
            _restarted = true;
            
            if ( connected ) {
                _sent[ interface.name() ] = false;
            } else {
                _sent.erase( interface.name() );
            }


            // This is called twice upon disconnect???? Really weird - maybe netmg issue?
            return _initialize( interface.name() );
        }

        virtual bool addInterface( const Interface& interface ) {
            if ( manages( interface ) ) {
                return false;
            }

            _nextNode = interface.name();
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

        virtual std::string name() const override { return "traversal-election"; }

        virtual std::string info() const override {
            std::string str = Protocol::info();
            std::stringstream ss;
            ss << "; leader id: " << _leaderId << " leader address: " << _leaderAddress;
            return str  + ss.str();
        }
    };
}