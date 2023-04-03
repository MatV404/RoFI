#include <iostream>
#include <lwip++.hpp>
#include <lwip/udp.h>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <networking/networkManager.hpp>
#include <networking/protocols/simpleReactive.hpp>
#include <invitation.hpp>
#include <query.hpp>
#include <atoms/units.hpp>
#include <random>

using namespace rofi::hal;
using namespace rofi::net;
using namespace rofi::leadership;
using namespace std::chrono_literals;

int NODE_STATE_CHANGE_CHANCE = -1;

Ip6Addr createAddress( int id ) {
    std::stringstream ss;
    ss << "fc07:0:0:";
    ss << id;
    ss << "::1";

    return Ip6Addr( ss.str() );
}

std::pair< void*, int > calcTask() {
    std::cout << "Calctask\n";
    int task = 1;
    return std::pair< void*, int >( &task, sizeof( int ) * 1 );
}

void getTask ( void* task, int size ) {
    std::cout << "getTask\n";
    // std::cout << "Task: " << *as< int* >( task ) << "\n";
}


bool workStop = false;

void stopWork() {
    std::cout << "Stopping work\n";
    workStop = true;
    std::cout << "Stopped work\n";
}

void testInvitation( NetworkManager& netmg, int id, Ip6Addr& addr ) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, 100);

    std::vector< Ip6Addr > addresses;
    addresses.push_back( ( createAddress( 1 ) ) );
    addresses.push_back( ( createAddress( 2 ) ) );
    addresses.push_back( ( createAddress( 3 ) ) );
    addresses.push_back( ( createAddress( 4 ) ) );
    addresses.push_back( ( createAddress( 5 ) ) );
    addresses.push_back( ( createAddress( 6 ) ) );
    addresses.push_back( ( createAddress( 7 ) ) );


    InvitationElection election( id, addr, 7776, addresses, calcTask, getTask, stopWork, 1, 3 );
    election.setUp();
    election.start();

    while ( true ) {
        int result = distr(gen);
        if ( result <= NODE_STATE_CHANGE_CHANCE ) {
            std::cout << "Node shutting down\n";
            election.switchDown();
        }

        if ( !workStop && election.getLeader().second ) {
            std::cout << "Node " << id << " is currently performing work under leader " << election.getLeader().first << "\n";
            std::cout << netmg.routingTable();
        }

        if ( workStop ) {
            if ( election.getLeader().second ) {
                workStop = false;
            }
        }

        sleep( 3 );
    }
}

void testQuery( NetworkManager& net, int id, Ip6Addr& addr ) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, 100);

    std::set< Ip6Addr > addresses;
    for ( int i = 1; i <= 5; i++ ) {
        addresses.emplace( createAddress( i ) );
    }

    QueryElect election( net, addr, &addresses, 7776, 2 );
    election.start();
    election.setUp();

    int counter = 0;
    bool off = false;

    while ( true ) {
        counter++;
        int result = ( counter == 3 ) ? distr(gen) : 100;
        if ( off && counter == 3 ) {
            result = NODE_STATE_CHANGE_CHANCE;   
        }
        if ( result <= NODE_STATE_CHANGE_CHANCE ) {
            std::cout << "Switching off\n";
            counter = 0;
            off = !off;
            election.switchDown();
        } else if ( !election.isDown() ) {
            std::cout << "My (" << addr << ") leader: " << election.leader() << "\n";
            std::cout << net.routingTable();
        }
        
        sleep( 3 );
    }
}

void testTolerant( bool invitationTest ) {
    std::cout << "Starting crash tolerant election test\n";
    tcpip_init( nullptr, nullptr );

    int id = RoFI::getLocalRoFI().getId();
    std::cout << "This module is: " << id << "\n";

    NetworkManager net( RoFI::getLocalRoFI() );

    Ip6Addr addr = createAddress( id );

    net.addAddress( addr, 80, net.interface( "rl0" ) );
    std::cout << "Current Address: " << net.interface( "rl0" ).getAddress().front().first << "\n";
    net.setUp();

    auto proto = net.addProtocol( SimpleReactive() );
    net.setProtocol( *proto );

    if ( invitationTest ) {
        testInvitation( net, id, addr );
        return;
    }

    testQuery( net, id, addr );
}