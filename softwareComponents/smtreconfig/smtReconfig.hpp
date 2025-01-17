#pragma once
#include <z3++.h>
#include <Configuration.h>
#include <array>

#include "mixins.hpp"

namespace rofi::smtr {

using ModuleIdx = int;

struct Parameters {
    enum class StepSize { Step90, Continuous };

    StepSize stepSize = StepSize::Continuous;
    bool shoeLimitConstain = false;
    bool connectorLimitConstrain = false;
    bool simplify = false;
};

struct Context {
    Parameters cfg;
    z3::context ctx;

    z3::expr sqrt2, sqrt3;

    Context( Parameters c = {} ):
        cfg( c ), ctx(), sqrt2( ctx.real_const( "sqrt2" ) ),
        sqrt3( ctx.real_const( "sqrt3" ) )
    {}

    z3::expr constraints() const {
        return sqrt2 * sqrt2 == 2 && sqrt3 * sqrt3 == 3;
    }
};

struct Shoe {
    z3::expr x, y, z; // Shoe center position
    z3::expr qa, qb, qc, qd; // Shoe orientation
};

struct SinCosAngle {
    z3::expr sin, sinhalf, cos, coshalf;
};

struct Module {
    SinCosAngle alpha, beta, gamma; // Joint position
    std::array< Shoe, 2 > shoes; // Shoes
    std::string prefix;
};

struct SmtConfiguration {
    template < typename T > using PerModule = std::vector< T >;
    template < typename T > using PerShoe = std::array< T, 2 >;
    template < typename T > using PerModuleShoe = PerModule< PerShoe < T > >;

    PerModuleShoe< PerModuleShoe < std::vector< z3::expr > > > connections;
        // All possible connections between shoe connectors
    std::vector< Module > modules; // All modules in the configuration

    const z3::expr& connection( ModuleIdx m, ShoeId ms, ConnectorId mc,
        ModuleIdx n, ShoeId ns, ConnectorId nc, Orientation o ) const
    {
        assert( m != n );
        if ( m > n ) {
            return connection( n, ns, nc, m, ms, mc, o );
        }
        assert( connections.size() > m );
        assert( connections[ m ][ ms ].size() > n - m - 1 );
        assert( connections[ m ][ ms ][ n - m - 1 ][ ns ].size() == 36 );
        return connections[ m ][ ms ][ n - m - 1 ][ ns ][ 3 * 4 * mc  + 4 * nc + o ];
    }

    z3::expr constraints( Context& ctx ) const;
};

void collectVar( const SinCosAngle& a, std::vector< z3::expr >& out );
void collectVar( const Shoe& s, std::vector< z3::expr >& out );
void collectVar( const Module& m, std::vector< z3::expr >& out );
void collectVar( const SmtConfiguration& c, std::vector< z3::expr >& out );

template < typename T >
std::vector< z3::expr > collectVar( const T& o ) {
    std::vector< z3::expr > ret;
    collectVar( o, ret );
    return ret;
}

std::vector< std::tuple< int, ShoeId, int, ShoeId > > allShoePairs( int count );
auto allShoeConnections() ->
    std::vector< std::tuple< ConnectorId, ConnectorId, Orientation > >;
auto allConnectors( int count ) ->
    std::vector< std::tuple< ModuleIdx, ShoeId, ConnectorId > >;

Context buildContext();
SmtConfiguration buildConfiguration( Context& ctx,
    const Configuration& cfg, int cfgId );

std::pair< z3::expr, std::vector< SmtConfiguration > > reconfig( Context& ctx,
    int len, const Configuration& init, const Configuration target );

z3::expr phiValid( Context& ctx, const SmtConfiguration& cfg );
z3::expr phiConsistent( Context& ctx, const SmtConfiguration& cfg );
z3::expr phiNoIntersect( Context& ctx, const SmtConfiguration& cfg );
z3::expr phiIsConnected( Context& ctx, const SmtConfiguration& cfg );
z3::expr phiShoeConsistent( Context& ctx, const SmtConfiguration& cfg );
z3::expr phiSinCos( Context& ctx, const SmtConfiguration& cfg );
z3::expr phiConnectorConsistent( Context& ctx, const SmtConfiguration& cfg );
z3::expr phiEqual( Context& ctx, const SmtConfiguration& smtCfg,
        const Configuration& cfg );
z3::expr phiRootModule( Context& ctx, const SmtConfiguration& cfg, int moduleIdx );
z3::expr phiEqualJoints( Context& ctx, const SmtConfiguration& a,
        const SmtConfiguration& b );
z3::expr phiStepConnect( Context& ctx, const SmtConfiguration& a,
        const SmtConfiguration& b );
z3::expr phiStepDisconnect( Context& ctx, const SmtConfiguration& a,
        const SmtConfiguration& b );
z3::expr phiStepRotate( Context& ctx, const SmtConfiguration& a,
        const SmtConfiguration& b );
z3::expr phiStep( Context& ctx, const SmtConfiguration& a,
        const SmtConfiguration& b );

} // rofi::smtr