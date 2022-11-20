#include <isoreconfig/isomorphic.hpp>
#include <cassert>

namespace rofi::isoreconfig {

using namespace rofi::configuration;

Positions decomposeUniversalModule( 
    const Module& mod )
{
    // <mod> must be a UniversalModule
    assert( mod.type == ModuleType::Universal );

    std::span<const Component> comps = mod.components();
    Positions result;

    for ( int compIndex = 0; compIndex < 6; ++compIndex )
    {
        // First six components of UniversalModule are Roficoms
        assert( comps[ compIndex ].type == ComponentType::Roficom );

        Matrix absCompPos = comps[ compIndex ].getPosition();
        // Component position of a UniversalModule is always in the center 
        // of the shoe it belongs to; to get the "actual" (visual) position,
        // we want to move it by half a unit on -X
        result.push_back( absCompPos * matrices::translate( { -0.5, 0, 0 } ) );
    } 

    return result;
}

std::array< Positions, 2 > decomposeRofiWorld( const RofiWorld& rw )
{
    rw.isValid().get_or_throw_as< std::logic_error >();

    std::array< Positions, 2 > result;

    // Decompose modules
    for ( const auto& /*RofiWorld::ModuleInfo*/ modInf : rw.modules() )
        for ( const Matrix& pos : decomposeUniversalModule( *modInf.module ) )
            result[0].push_back( pos );

    // Decompose connections
    for ( const RoficomJoint& connection : rw.roficomConnections() )
    {
        Matrix pos = connection.getSourceModule( rw ).components()[connection.sourceConnector].getPosition();
        // Connection position is in the center of the connected module,
        // so it must be translated by half a unit
        result[1].push_back( pos * matrices::translate( { -0.5, 0, 0 } ) );
    }

    return result;
}

Matrix centroid( const RofiWorld& rw )
{
    return pointToPos( centroid( decomposeRofiWorld( rw )[0] ));
}


Positions cloudToPositions( const Cloud& cop )
{
    Positions result;
    for ( const Point& p : cop ) result.push_back( pointToPos( p ) );
    return result;
}

Cloud positionsToCloud( const Positions& poss )
{
    Cloud result;
    for ( const Matrix& pos : poss ) result.push_back( posToPoint( pos ) );
    return result;
}

bool equalShape( const RofiWorld& rw1, const RofiWorld& rw2 )
{
    auto comparePoints = [&]( const Point& p1, const Point& p2 ) 
        { return p1(0) < p2(0) || ((p1(0) == p2(0) && p1(1) < p2(1)) || (p1(0) == p2(0) && p1(1) == p2(1) && p1(2) < p2(2))); };

    std::array< Positions, 2 > positions1 = decomposeRofiWorld( rw1 );
    std::array< Positions, 2 > positions2 = decomposeRofiWorld( rw2 );

    // Cloud cop1 = positionsToCloud( positions1[0] );
    // Cloud cop2 = positionsToCloud( positions2[0] );

    // std::cout << "cop1:\n";
    // cloudToScore( cop1 ).print();

    // std::cout << "cop2:\n";
    // cloudToScore( cop2 ).print();

    // std::sort( cop1.begin(), cop1.end(), comparePoints );
    // std::sort( cop2.begin(), cop2.end(), comparePoints );

    // std::cout << "cop1:\n";
    // cloudToScore( cop1 ).print();

    // std::cout << "cop2:\n";
    // cloudToScore( cop2 ).print();

    // Merge module points and connection points into one cloud
    for ( const Matrix& pos : positions1[1] )
        positions1[0].push_back( pos );
    for ( const Matrix& pos : positions2[1] )
        positions2[0].push_back( pos );

    // cop1 = positionsToCloud( positions1[0] );
    // cop2 = positionsToCloud( positions2[0] );

    // std::sort( cop1.begin(), cop1.end(), comparePoints );
    // std::sort( cop2.begin(), cop2.end(), comparePoints );

    // std::cout << "cop1:\n";
    // cloudToScore( cop1 ).print();

    // std::cout << "cop2:\n";
    // cloudToScore( cop2 ).print();

    return isometric( positionsToCloud( positions1[0] ), positionsToCloud( positions2[0] ) );
}


} // namespace rofi::isoreconfig
