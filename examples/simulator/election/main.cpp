#include <networking/networkManagerCli.hpp>
#include "lwip++.hpp"


#include <echo2.hpp>
#include <networking/protocols/rrp.hpp>
// #include <newTraverse.hpp>
#include <networking/protocols/simpleReactive.hpp>
#include <networking/protocols/leaderElect.hpp>

#include <lwip/udp.h>
#include <iostream>
#include <string>

#include "crashTolerance.cpp"
#include "reactive.cpp"

using namespace rofi::hal;
using namespace rofi::net;

const char* getAddr( int id ) {
    std::ostringstream addr;
    addr << "fc07:0:0:" << id << ":1";
    return addr.str().c_str();
}

void onMessage(  void*, struct udp_pcb*, struct pbuf* p, const ip6_addr_t* addr, u16_t ) {
    if ( !p || !addr )
        return;

    auto packet = PBuf::own( p );
    std::cout << "Got message from " << Ip6Addr( *addr ) << "; msg: " << packet.asString() << "\n";
    std::cout << "> " << std::flush;
}

udp_pcb* setUpListener() {
    udp_pcb* pcb = udp_new();
    if ( !pcb ) {
        throw std::runtime_error( "pcb is null" );
    }
    udp_bind( pcb, IP6_ADDR_ANY, 7777 );
    udp_recv( pcb, onMessage, nullptr );

    return pcb;
}

void handleConnector( bool connect ) {
    int connectorIdx;
    auto rofi = RoFI::getLocalRoFI();
    std::cout << "which connector do you want to disconnect? (0 - "
              << rofi.getDescriptor().connectorCount << "): ";
    std::cin >> connectorIdx;
    if ( connectorIdx >= 0 && connectorIdx < rofi.getDescriptor().connectorCount ) {
        if ( connect ) {
            rofi.getConnector( connectorIdx ).connect();
        } else {
            rofi.getConnector( connectorIdx ).disconnect();
        }
    } else {
        std::cout << "index out of range! Ignoring..." << std::endl;
    }
}

int main( void ) {
    testTolerant( true );
    // testProtocol( true );
    return 0;
}