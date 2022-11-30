#include <isoreconfig/isomorphic.hpp>
#include <isoreconfig/geometry.hpp>
#include <cassert>

namespace rofi::isoreconfig {

using namespace rofi::configuration;

Matrix pointMatrix( const Vector& pt )
{
    Matrix result( arma::eye( 4, 4 ) );
    result.col(3) = pt;
    result(3,3) = 1;
    return result;
}

Points decomposeModule( const Module& mod )
{
    Points result;

    for ( const Component& comp : mod.components() )
    {
        if ( comp.type != ComponentType::Roficom ) 
            continue;

        Matrix posMat = comp.getPosition() * matrices::translate( { -0.5, 0, 0 } );
        // Center of roficoms of a module is in the center 
        // of the shoe it belongs to; to get the "actual" (visual) position,
        // move it by half a unit in the direction the roficom is facing (X axis)
        result.push_back( posMat.col(3) );
    } 

    return result;
}

std::array< Points, 2 > decomposeRofiWorld( const RofiWorld& rw )
{
    rw.isValid().get_or_throw_as< std::logic_error >();

    std::array< Points, 2 > result;

    // Decompose modules
    for ( const auto& /*ModuleInfo*/ modInf : rw.modules() )
        for ( const Vector& pt : decomposeModule( *modInf.module ) )
            result[0].push_back( pt );

    // Decompose connections
    for ( const RoficomJoint& connection : rw.roficomConnections() )
    {
        Matrix posMat = connection.getSourceModule( rw ).components()[connection.sourceConnector].getPosition();
        posMat *= matrices::translate( { -0.5, 0, 0 } );
        // Connection position is in the center of the connected module,
        // so it must be translated by half a unit
        result[1].push_back( posMat.col(3) );
    }

    return result;
}

Vector centroid( const RofiWorld& rw )
{
    std::array< Points, 2 > pts = decomposeRofiWorld( rw );
    for ( const Vector& pt : pts[1] )
        pts[0].push_back( pt );
    return centroid( pts[0] );
}

Vector centroid( const Points& pts )
{
    assert( pts.size() >= 1 );

    Vector result = std::accumulate(
        ++pts.begin(), pts.end(), pts[0], 
        []( const Vector& pt1, const Vector& pt2 ){ return pt1 + pt2; } );

    result(3) = 1;
    for ( size_t i = 0; i < 3; ++i )
        result(i) /= double(pts.size());

    return result;
}

bool equalShape( const RofiWorld& rw1, const RofiWorld& rw2 )
{
    // Worlds with different number of modules or connections cannot have same shape
    if ( rw1.modules().size() != rw2.modules().size() ||
        rw1.roficomConnections().size() != rw2.roficomConnections().size() )
        return false;

    std::array< Points, 2 > pts1 = decomposeRofiWorld( rw1 );
    std::array< Points, 2 > pts2 = decomposeRofiWorld( rw2 );

    assert( pts1[0].size() == pts2[0].size() );
    assert( pts1[1].size() == pts2[1].size() );

    // Merge module points and connection points into one container
    for ( const Vector& pt : pts1[1] )
        pts1[0].push_back( pt );
    for ( const Vector& pt : pts2[1] )
        pts2[0].push_back( pt );

    assert( pts1[0].size() == pts2[0].size() );
    
    return isometric( Cloud( pts1[0] ), Cloud( pts2[0] ) );
}

} // namespace rofi::isoreconfig
