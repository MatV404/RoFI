#include <configuration/rofiworld.hpp>
#include <isoreconfig/isomorphic.hpp>
#include <dimcli/cli.h>

#include "common.hpp"

static auto command = Dim::Cli().command( "shape" )
    .desc( "Compare the shapes of two given configurations" );
static auto& firstFile = command.opt< std::string >( "<FILE>" )
    .desc("First configuration file");
static auto& secondFile = command.opt< std::string >( "<FILE>" )
    .desc("Second configuration file");

void shape( Dim::Cli &cli ) 
{
    auto world1 = parseRofiWorld( *firstFile );
    if ( !world1 ) {
        cli.fail( EXIT_FAILURE, "Error while parsing first world", world1.assume_error() );
        return;
    }
    auto world2 = parseRofiWorld( *secondFile );
    if ( !world2 ) {
        cli.fail( EXIT_FAILURE, "Error while parsing second world", world2.assume_error() );
        return;
    }

    if ( world1->modules().empty() || world2->modules().empty() )
        exit( int(world1->modules().size() + world2->modules().size()) );

    if ( world1->referencePoints().empty() )
        affixRofiWorld( *world1 );
    if ( world2->referencePoints().empty() )
        affixRofiWorld( *world2 );

    if ( auto valid = world1->validate(); !valid ) {
        cli.fail( EXIT_FAILURE, "First rofi world is invalid", valid.assume_error() );
        return;
    }
    if ( auto valid = world2->validate(); !valid ) {
        cli.fail( EXIT_FAILURE, "Second rofi world is invalid", valid.assume_error() );
        return;
    }

    if ( !rofi::isoreconfig::equalShape( *world1, *world2 ) ) {
        exit( EXIT_FAILURE ); // Avoid dimcli error message
    }
}

