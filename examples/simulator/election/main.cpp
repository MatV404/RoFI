#include <networking/networkManagerCli.hpp>
#include "lwip++.hpp"

#include <networking/protocols/rrp.hpp>
#include <networking/protocols/simpleReactive.hpp>

#include <lwip/udp.h>
#include <iostream>
#include <string>

#include "crashTolerance.cpp"
#include "reactive.cpp"

int main( void ) {
    testTolerant( true );
    // testProtocol( false );
    return 0;
}