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
    // if ( !rw.isPrepared() ) rw.prepare().get_or_throw_as< std::logic_error >();
    rw.isValid().get_or_throw_as< std::logic_error >();

    // [0] == module points, [1] == connection points
    std::array< Positions, 2 > result;

    // Decompose modules
    for ( const auto& /*ModuleInfo*/ modInf : rw.modules() )
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
    if ( rw1.modules().size() != rw2.modules().size() ||
        rw1.roficomConnections().size() != rw2.roficomConnections().size() )
        return false;

    std::array< Positions, 2 > positions1 = decomposeRofiWorld( rw1 );
    std::array< Positions, 2 > positions2 = decomposeRofiWorld( rw2 );

    assert( positions1[0] == positions2[0] );
    assert( positions1[1] == positions2[1] );

    // Merge module points and connection points into one cloud
    for ( const Matrix& pos : positions1[1] )
        positions1[0].push_back( pos );
    for ( const Matrix& pos : positions2[1] )
        positions2[0].push_back( pos );

    assert( positions1[0] == positions2[0] );

    return isometric( positionsToCloud( positions1[0] ), positionsToCloud( positions2[0] ) );
}


} // namespace rofi::isoreconfig
