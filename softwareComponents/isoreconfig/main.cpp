#include "dimcli/cli.h"

#include <configuration/rofiworld.hpp>
#include <configuration/joints.hpp>
#include <configuration/universalModule.hpp> 
#include <configuration/serialization.hpp>

#include <isoreconfig/isomorphic.hpp>
#include <isoreconfig/bfshapes.hpp>

using namespace rofi::configuration;
using namespace rofi::isoreconfig;

RofiWorld parseRofiWorld( const std::string& path )
{
    std::ifstream inputTarget;
    inputTarget.open( path );
    if (inputTarget.fail()) 
    {
        std::cerr << "Invalid path to rofiworld: '" << path << "'\n";
        exit(1);
    }
    RofiWorld result = readOldConfigurationFormat( inputTarget );
    
    const auto identity = arma::mat(4, 4, arma::fill::eye);
    assert( result.modules().size() > 0 );
    connect< RigidJoint >( (*result.modules().begin()).module->bodies().front(), { 0, 0, 0 }, identity );
    result.prepare().get_or_throw_as< std::runtime_error >();

    return result;
}

// void saveToFile( const RofiWorld& world, const std::string& path )
// {
//     auto jason = serialization::toJSON( world );
//     std::ofstream out(path);
//     // std::cout << jason.dump() << "\n";
//     out << jason.dump();
//     out.close();
// }

int main(int argc, char** argv) 
{
    Dim::Cli cli;
    auto & startPath = cli.opt<std::string>("start", "./start.in").desc("Starting configuration in valid format");
    auto & targetPath = cli.opt<std::string>("target", "./target.in").desc("Target configuration in valid format");
    auto & outputPath = cli.opt<std::string>("output", "./bfshapesOut/").desc("Directory to serialize the found path into");
    auto & step = cli.opt<int>("step", 90).desc("Degree of rotation for 1 step");
    if (!cli.parse(argc, argv))
        return cli.printError(std::cerr); // prints error and returns cli.exitCode()

    Reporter rep;

    std::vector< RofiWorld > result = bfsShapes( 
        parseRofiWorld( *startPath ),
        parseRofiWorld( *targetPath ),
        Angle::deg( *step ).rad(), rep );

    std::cout << rep.toString();

    for ( size_t i = 0; i < result.size(); ++i )
    {
        std::stringstream path;
        path << *outputPath << i << ".json";
        // std::cout << "writing to: " << path.str() << "\n";
        saveToFile( result[i], path.str() );
    }
}

