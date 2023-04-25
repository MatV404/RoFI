#pragma once

#include <networking/networkManager.hpp>
#include <LRHelper.hpp>
#include <set>
#include <thread>
#include <functional>

namespace rofi::leadership {
    using namespace rofi::net;

    class LRElect {
        NetworkManager& _net;
        const Ip6Addr& _myAddr;
        Ip6Addr _leader;

        bool _down = false;

        unsigned int _timeJoined;
        unsigned int _minTimeJoined;
        unsigned int _sequenceNumber = 0;
        unsigned int _period;

        bool _leaderContact = false;

        void _received( Ip6Addr addr, unsigned int logTime ) {
            if ( ( logTime < _minTimeJoined ) 
                || ( logTime == _minTimeJoined && addr < _leader ) ) {
                _leader = addr;
                _minTimeJoined = logTime;
            }

            if ( addr == _leader ) {
                _leaderContact = true;
            }
        }

        void _increaseTimeJoined() {
            _timeJoined++;
            if ( _leader == _myAddr) {
                _minTimeJoined = _timeJoined;
            }
        }

        void _leaderFailure() {
            _leader = _myAddr;
            _minTimeJoined = _timeJoined;
        }

        void _periodic() {
            while ( true ) {
                if ( _down ) {
                    continue;
                }
                if ( _leader == _myAddr ) {
                    for ( const Interface& interface : _net.interfaces() ) {
                        if ( interface.name() == "rl0" ) {
                            continue;
                        }
                        PBuf packet = PBuf::allocate( Ip6Addr::size() + 2 * sizeof( unsigned int ) );
                        as< Ip6Addr >( packet.payload() ) = _myAddr;
                        as< unsigned int >( packet.payload() + Ip6Addr::size() ) = _timeJoined;
                        as< unsigned int >( packet.payload() + Ip6Addr::size() + sizeof( unsigned int ) ) = _sequenceNumber;
                        if ( ! const_cast< Interface& >( interface ).sendProtocol( _net.getProtocol( "lr-helper" )->address(), std::move( packet ) ) ) {
                            assert( false && "failed to send a message to the helper protocol. Something went wrong." );
                        }
                    }
                    _sequenceNumber++;
                } else {
                    if ( !_leaderContact ) {
                        _leaderFailure();
                    }

                    _leaderContact = false;
                }
                sleep( _period );
            }
        }

    public:
        LRElect( NetworkManager& net, const Ip6Addr& addr, unsigned int period ) : _net( net ), _myAddr( addr ),  _leader( addr ) {
            _timeJoined = 0;
            _minTimeJoined = 0;
            _period = period;
            std::function< void ( const Ip6Addr, unsigned int ) > fun = [ this ]( const Ip6Addr address, unsigned int logTime ){ _received( address, logTime); };
            Protocol* prot = net.addProtocol( LRHelper( addr, fun, [ this ](){ _increaseTimeJoined(); } ) );
            net.setProtocol( *prot );
        }

        void start( int id ) {
            sleep( id );
            std::thread thread{ [ this ]() {
                this->_periodic();
            } };
            thread.detach();
        }

        const Ip6Addr& getLeader() {
            return _leader;
        }

        void switchDown() {
            _down = !_down;
        }
    };
}