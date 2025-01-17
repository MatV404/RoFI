#include "voxel_reconfig.hpp"

#include <catch2/catch.hpp>

#include "configuration/rofiworld.hpp"
#include "configuration/universalModule.hpp"
#include "voxel.hpp"


using namespace rofi::configuration;

TEST_CASE( "No move" )
{
    SECTION( "Single module" )
    {
        auto world = RofiWorld();

        auto & um = world.insert( UniversalModule( 1, 0_deg, 0_deg, 0_deg ) );
        connect< RigidJoint >( um.connectors()[ 0 ], Vector{ 0, 0, 0 }, matrices::identity );
        REQUIRE( world.validate() );

        auto voxelWorld = rofi::voxel::VoxelWorld::fromRofiWorld( world );
        REQUIRE( voxelWorld );

        auto result = rofi::voxel::voxel_reconfig( *voxelWorld, *voxelWorld );
        REQUIRE( result );

        CHECK( result->size() == 1 );
    }

    SECTION( "Two modules" )
    {
        auto world = RofiWorld();

        auto & um1 = world.insert( UniversalModule( 1, 0_deg, 0_deg, 0_deg ) );
        auto & um2 = world.insert( UniversalModule( 2, 0_deg, 0_deg, 0_deg ) );
        connect( um1.connectors()[ 4 ], um2.connectors()[ 1 ], roficom::Orientation::East );
        connect< RigidJoint >( um1.connectors()[ 0 ], Vector{ 0, 0, 0 }, matrices::identity );
        REQUIRE( world.validate() );

        auto voxelWorld = rofi::voxel::VoxelWorld::fromRofiWorld( world );
        REQUIRE( voxelWorld );

        auto result = rofi::voxel::voxel_reconfig( *voxelWorld, *voxelWorld );
        REQUIRE( result );

        CHECK( result->size() == 1 );
    }
}

TEST_CASE( "One move" )
{
    SECTION( "Single module" )
    {
        auto initWorld = RofiWorld();
        auto & initUm = initWorld.insert( UniversalModule( 1, 0_deg, 0_deg, 0_deg ) );
        connect< RigidJoint >( initUm.connectors()[ 0 ], Vector{ 0, 0, 0 }, matrices::identity );
        REQUIRE( initWorld.validate() );
        auto initVoxelWorld = rofi::voxel::VoxelWorld::fromRofiWorld( initWorld );
        REQUIRE( initVoxelWorld );

        auto goalWorld = RofiWorld();
        auto & goalUm = goalWorld.insert( UniversalModule( 1, 0_deg, 0_deg, 0_deg ) );
        connect< RigidJoint >( goalUm.connectors()[ 0 ], Vector{ 0, 0, 0 }, matrices::identity );
        SECTION( "Move alpha" )
        {
            goalUm.setAlpha( GENERATE( 90_deg, -90_deg ) );
        }
        SECTION( "Move beta" )
        {
            goalUm.setBeta( GENERATE( 90_deg, -90_deg ) );
        }
        SECTION( "Move gamma" )
        {
            goalUm.setGamma( GENERATE( 90_deg, -90_deg ) );
        }

        REQUIRE( goalWorld.validate() );
        auto goalVoxelWorld = rofi::voxel::VoxelWorld::fromRofiWorld( goalWorld );
        REQUIRE( goalVoxelWorld );


        auto result = rofi::voxel::voxel_reconfig( *initVoxelWorld, *goalVoxelWorld );
        REQUIRE( result );

        CHECK( result->size() == 2 );
    }
}
