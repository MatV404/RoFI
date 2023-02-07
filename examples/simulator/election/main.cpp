#include <networking/networkManagerCli.hpp>
#include "lwip++.hpp"


#include <echo.hpp>
#include <simpleElect.hpp>
#include <traverseElect.hpp>

#include <lwip/udp.h>
#include <iostream>
#include <string>

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
    std::cout << "Starting leadership election demo\n";
    tcpip_init(nullptr, nullptr);
    auto local = rofi::hal::RoFI::getLocalRoFI();
    NetworkManager net( local );
    NetworkManagerCli netcli( net );
    int id = local.getId();
    std::cout << "ID: " + std::to_string( id ) + "\n";

    if ( id == 1 ) {
        net.addAddress( "fc07:0:0:1::1"_ip, 80, net.interface( "rl0" ) );
    } else if ( id == 2 ) {
        net.addAddress( "fc07:0:0:2::1"_ip, 80, net.interface( "rl0" ) );
    } else if ( id == 3 ) {
        net.addAddress( "fc07:0:0:3::1"_ip, 80, net.interface( "rl0" ) );
    } else if ( id == 4 ) {
        net.addAddress( "fc07:0:0:4::1"_ip, 80, net.interface( "rl0" ) );
    } else {
        throw std::runtime_error( "more than 4 bots!" );
    }

    std::cout << "address: " << net.interface( "rl0" ).getAddress().front().first << std::endl;
    sleep( 2 );
    net.setUp();

    auto* pcb = setUpListener();

    // To pick an election protocol, simply comment / uncomment one of these lines.
    // auto result = net.addProtocol( DemoElection( id, "fc07:b::a"_ip, 96 ));
    // auto result = net.addProtocol( EchoElection( id, "fc07:b::a"_ip, 96 ) );
    auto result = net.addProtocol( TraverseElection( id, "fc07:b::a"_ip, 96 ) );

    net.setProtocol( *result );

    std::string line;
    std::cout << "> ";

    while ( std::getline( std::cin, line ) ) {
        if ( line.empty() ) {
            
            continue;
        } else if ( line == "end" || line == "q" || line == "quit" ) {
            break;
        }
        
        try {
            if ( line == "connect" || line == "disconnect" ) {
                handleConnector( line == "connect" );
            } else if ( netcli.command( line ) ) {
            } else {
                std::cout << "Do nothing.";
            }
        } catch ( const std::exception& e ) {
            std::cout << "Bad input: " << e.what() << std::endl;
        }

        std::cout << "> ";
    }
    std::cout << "Ending election example\n";
    return 0;
}