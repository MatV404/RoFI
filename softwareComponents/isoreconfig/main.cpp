// #include "BFS.hpp"
#include "dimcli/cli.h"
#include <isoreconfig/isomorphic.hpp>
#include <isoreconfig/geometry.hpp>
#include <isoreconfig/isoreconfig.hpp>
#include <configuration/rofiworld.hpp>
#include <configuration/joints.hpp>
#include <configuration/universalModule.hpp> 
#include <configuration/serialization.hpp>

using namespace rofi::configuration;
using namespace rofi::isoreconfig;

RofiWorld parseRofiWorld( const std::string& path )
{
    std::ifstream inputTarget;
    inputTarget.open( path );
    if (inputTarget.fail()) 
    {
        std::cerr << "Invalid path to rofiworld\n";
        exit(1);
    }
    RofiWorld result = readOldConfigurationFormat( inputTarget );
    
    const auto identity = arma::mat(4, 4, arma::fill::eye);
    connect< RigidJoint >( result.getModule(0)->bodies()[ 0 ], { 0, 0, 0 }, identity );
    result.prepare().get_or_throw_as< std::runtime_error >();

    return result;
}

void saveToFile( const RofiWorld& world, const std::string& path )
{
    auto jason = serialization::toJSON( world );
    std::ofstream out(path);
    std::cout << jason.dump() << "\n";
    out << jason.dump();
    out.close();
}

void testOne90();

int main(int argc, char** argv) 
{
    RofiWorld world;
    // add universal module with id 42 in the default state
    auto& m1 = world.insert( UniversalModule( 42, 0_deg, 0_deg, 0_deg ) );
    // add universal module with id 42 with beta set to 45 degrees and gamma to 90 degrees
    auto& m2 = world.insert( UniversalModule( 66, 0_deg, 45_deg, 90_deg ) );

    // connect A+X of the universal module with id = 42 to A-X of UM with id = 66
    connect( m1.connectors()[ 2 ], m2.connectors()[ 0 ], roficom::Orientation::North );
    // fix the position of the `shoe A` in { 0, 0, 0 }
    const auto identity = arma::mat(4, 4, arma::fill::eye);
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    world.prepare().get_or_throw_as< std::runtime_error >();;
    decomposeRofiWorld( world );

    RofiWorld world1, world2;
    m1 = world1.insert( UniversalModule( 42, 45_deg, 0_deg, 0_deg ) );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    // add universal module with id 42 with beta set to 45 degrees and gamma to 90 degrees
    m2 = world2.insert( UniversalModule( 66, 0_deg, 45_deg, 0_deg ) );
    connect< RigidJoint >( m2.bodies()[ 0 ], { 0, 0, 0 }, identity );

    world1.prepare().get_or_throw_as< std::runtime_error >();;
    world2.prepare().get_or_throw_as< std::runtime_error >();;

    assert( equalShape( world1, world2 ) );

    // Test all possible shapes of one module with 90 degree step
    testOne90();

    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/TripleA1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/TripleA1.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/TripleA2.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/TripleA1.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/TripleA1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/TripleA2.in" ) 
    ));
    assert( !equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/TripleA1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/triple.in" ) 
    ));

    // Test BFS with shapes

    Dim::Cli cli;
    auto & startPath = cli.opt<std::string>("start", "./start.in").desc("Starting configuration in valid format");
    auto & targetPath = cli.opt<std::string>("target", "./target.in").desc("Target configuration in valid format");
    auto & step = cli.opt<int>("step", 90).desc("Degree of rotation for 1 step");
    if (!cli.parse(argc, argv))
        return cli.printError(std::cerr); // prints error and returns cli.exitCode()

    std::vector< RofiWorld > result = bfsShapes( 
        parseRofiWorld( *startPath ),
        parseRofiWorld( *targetPath ),
        Angle::deg( *step ).rad(), 1 );

    for ( size_t i = 0; i < result.size(); ++i )
    {
        std::stringstream path;
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsResult/" << i << ".json";
        std::cout << "writing to: " << path.str() << "\n";
        saveToFile( result[i], path.str() );
    }

    /* Dim::Cli cli;
    auto & startPath = cli.opt<std::string>("start", "./start.in").desc("Starting configuration in valid format");
    auto & targetPath = cli.opt<std::string>("target", "./target.in").desc("Target configuration in valid format");
    auto & step = cli.opt<int>("step", 90).desc("Degree of rotation for 1 step");
    auto & bound = cli.opt<int>("bound", 1).desc("Bound");
    auto & showStats = cli.opt<bool>("stats").desc("Show statistics of the BFS search");
    if (!cli.parse(argc, argv))
        return cli.printError(std::cerr); // prints error and returns cli.exitCode()
    
    Configuration start;
    Configuration target;
    BFSReporter reporter;

    // Read start configuration
    std::ifstream inputStart;
    inputStart.open(*startPath);
    if (inputStart.fail()) 
    {
        std::cerr << "Invalid path to start configuration: " << *startPath << "\n";
        exit(1);
    }
    if (!IO::readConfiguration(inputStart, start)) 
    {
        std::cerr << "Start configuration is not in valid format" << "\n";
        exit(1);
    }
    if (!start.isValid()) 
    {
        std::cerr << "Start configuration is not valid" << "\n";
        exit(1);
    }

    // Read target configuration
    std::ifstream inputTarget;
    inputTarget.open(*targetPath);
    if (inputTarget.fail()) 
    {
        std::cerr << "Invalid path to target configuration" + *targetPath;
        exit(1);
    }
    if (!IO::readConfiguration(inputTarget, target)) 
    {
        std::cerr << "Target configuration is not in valid format";
        exit(1);
    }
    if (!target.isValid()) 
    {
        std::cerr << "Start configuration is not valid";
        exit(1);
    }

    // Get configs from start to target
    std::vector<Configuration> result = BFS(start, target, *step, *bound, reporter);

    // Write the result of BFS search
    std::cout << IO::toString(result);

    // Show BFS stats
    if (*showStats) {
        std::cout << reporter.toString();
    }

    int i = 0;

    for ( const Configuration &c : seen ) {
        std::string fileName = "./configs/" + std::to_string( i ) + ".in";
        std::cout << fileName << "\n";
        std::ofstream myFile( fileName );
        myFile << IO::toString( c ) << "\n";
        i++;
    }

    std::cout << seen.size() << "\n"; */

}

void testOne90()
{
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A2.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A3.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A4.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A2.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A2.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A2.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A3.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A2.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A4.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A3.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A3.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A3.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A4.in" ) 
    ));
    assert( equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A4.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A4.in" ) 
    ));
    assert( !equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/B1.in" ) 
    ));
    assert( !equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/C1.in" ) 
    ));
    assert( !equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/D1.in" ) 
    ));
    assert( !equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/E1.in" ) 
    ));
    assert( !equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/F1.in" ) 
    ));
    assert( !equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/G1.in" ) 
    ));
    assert( !equalShape( 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/A1.in" ), 
        parseRofiWorld( "/home/jarom/RoFI/softwareComponents/isoreconfig/configs/H1.in" ) 
    ));
}
