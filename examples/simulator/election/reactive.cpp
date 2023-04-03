#include <iostream>
#include <lwip++.hpp>
#include <newEcho.hpp>
#include <traverseElect.hpp>

using namespace rofi::hal;
using namespace rofi::net;

void electionStatusUpdate( bool elected, ElectionStatus nodeStatus ) {
    if ( elected ) {
        std::cout << "Election finished.\n";
    }
    std::cout << "NodeStatus is : ";
    if ( nodeStatus == ElectionStatus::UNDECIDED ) {
        std::cout << "Undecided\n";
    }
        std::cout << ( ( nodeStatus == ElectionStatus::FOLLOWER ) ? "Follower\n" : "Leader\n" );
}

void testProtocol( bool echoTest ) {
    auto localRoFI = RoFI::getLocalRoFI();
    int id = localRoFI.getId();
    NetworkManager net( localRoFI );

    Ip6Addr addr = "fc07:b::a"_ip;

    Protocol* protocol = ( echoTest ) ? net.addProtocol( EchoElection( id, addr, 80, electionStatusUpdate ) ) : net.addProtocol( TraverseElection( id, addr, 80 ) );
    net.setProtocol( *protocol );

    while ( true ) {
        std::cout << net.routingTable();
        sleep( 3 );
    }
}