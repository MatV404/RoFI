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
    /** To test these algorithms, uncomment / change the respective line.
     * testTolerant - true to test Invitation, false to test LR
     * testProtocol - true to test Echo, false to test Traverse
    */
    // testTolerant( true );
    testProtocol( true );
    return 0;
}