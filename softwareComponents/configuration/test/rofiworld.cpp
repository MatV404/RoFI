#include <catch2/catch.hpp>

#include <configuration/pad.hpp>
#include <configuration/rofiworld.hpp>
#include <configuration/universalModule.hpp>
#include <configuration/unknownModule.hpp>

namespace {

using namespace rofi::configuration;
using namespace rofi::configuration::roficom;
using namespace rofi::configuration::matrices;

TEST_CASE( "UnknownModule (base Module) Test" ) {
    auto m = UnknownModule( { Component{ ComponentType::Roficom, {}, {}, nullptr } }, 1, {}, 42 );
    CHECK( m.bodies().size() == 0 );
    CHECK( m.components().size() == 1 );
    CHECK( m.connectors().size() == 1 );
    CHECK( m.getId() == 42 );
    CHECK( m.setId( 66 ) );
    CHECK( m.getId() == 66 );
}

TEST_CASE( "Universal Module Test" ) {
    SECTION( "Creation" ) {
        auto um = UniversalModule( 0, 0_deg, 0_deg, 0_deg );
        CHECK( um.components().size() == 10 );
        REQUIRE( um.prepare() );
        REQUIRE( um.getOccupiedRelativePositions().size() == 2 );
        CHECK( equals( um.getOccupiedRelativePositions()[ 0 ], identity ) );
        CHECK( equals( center( um.getOccupiedRelativePositions()[ 1 ] ), { 0, 0, 1, 1 } ) );

        CHECK( um.getConnector( "A-Z" ).parent == &um );
    }

    SECTION( "roficomConnections" ) {
        auto um = UniversalModule( 0, 0_deg, 0_deg, 0_deg );
        CHECK( um.connectors().size() == 6 );

        for ( size_t i = 0; i < um.connectors().size(); i++ ) {
            INFO( "Connector number: " << i );
            CHECK( um.connectors()[ i ].type == ComponentType::Roficom );
        }

        REQUIRE( um.components().size() >= 6 );
        for ( size_t i = 0; i < um.components().size(); i++ ) {
            INFO( "Number of connectors: " << um.connectors().size() );
            INFO( "Component number: " << i );
            if ( i < um.connectors().size() ) {
                CHECK( um.components()[ i ].type == ComponentType::Roficom );
            } else {
                CHECK( um.components()[ i ].type != ComponentType::Roficom );
            }
        }
    }

    SECTION( "Position - default" ) {
        auto um = UniversalModule( 0, 0_deg, 0_deg, 0_deg );
        // A part
        CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
        CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
        CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
        // B part
        // -X
        CHECK( equals( um.getComponentRelativePosition( 3 ), translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 0, 1 } )
                                                   * rotate( M_PI, { 1, 0, 0 } ) ) );
        // +X
        CHECK( equals( um.getComponentRelativePosition( 4 ), translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 1, 0 } )
                                                   * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        // -Z
        CHECK( equals( um.getComponentRelativePosition( 5 ), translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 1, 0 } )
                                                   * rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        // Body
        CHECK( equals( um.getComponentRelativePosition( 8 ), translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 1, 0 } ) ) );
        // Shoe
        CHECK( equals( um.getComponentRelativePosition( 9 ), { { -1, 0,  0, 0 }
                                                     , {  0, 1,  0, 0 }
                                                     , {  0, 0, -1, 1 }
                                                     , {  0, 0,  0, 1 } } ) );
    }

    SECTION( "Position - rotated gamma" ) {
        SECTION( "degrees" ) {
            auto um = UniversalModule( 0, 0_deg, 0_deg, 90_deg );
            // A part
            CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
            // B part
            // -X
            CHECK( equals( um.getComponentRelativePosition( 3 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // +X
            CHECK( equals( um.getComponentRelativePosition( 4 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                    * rotate( M_PI, { 0, 1, 0 } ) * rotate( M_PI, { 0, 0, 1 } )
                                                    * rotate( M_PI, { 1, 0, 0 } ) ) );
            // -Z
            CHECK( equals( um.getComponentRelativePosition( 5 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } ) * rotate( - M_PI_2, { 0, 1, 0 } )
                                                        * rotate( M_PI, { 1, 0, 0 } ) ) );
            // Body
            CHECK( equals( um.getComponentRelativePosition( 8 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                    * rotate( M_PI, { 0, 1, 0 } ) ) );
            // Shoe
            CHECK( equals( um.getComponentRelativePosition( 9 ), { {  0, -1,  0, 0 } // shoeB
                                                        , { -1,  0,  0, 0 }
                                                        , {  0,  0, -1, 1 }
                                                        , {  0,  0,  0, 1 } } ) );
        }
    }

    SECTION( "radians" ) {
        auto um = UniversalModule( 0, 0_rad, 0_rad, 1.57079632679489661923_rad ); // 0, 0, M_PI_2
        // A part
        CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
        CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
        CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
        // B part
        // -X
        CHECK( equals( um.getComponentRelativePosition( 3 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                    * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        // +X
        CHECK( equals( um.getComponentRelativePosition( 4 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                   * rotate( M_PI, { 0, 1, 0 } ) * rotate( M_PI, { 0, 0, 1 } )
                                                   * rotate( M_PI, { 1, 0, 0 } ) ) );
        // -Z
        CHECK( equals( um.getComponentRelativePosition( 5 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                    * rotate( M_PI, { 0, 1, 0 } ) * rotate( - M_PI_2, { 0, 1, 0 } )
                                                    * rotate( M_PI, { 1, 0, 0 } ) ) );
        // Body
        CHECK( equals( um.getComponentRelativePosition( 8 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                   * rotate( M_PI, { 0, 1, 0 } ) ) );
        // Shoe
        CHECK( equals( um.getComponentRelativePosition( 9 ), { {  0, -1,  0, 0 } // shoeB
                                                     , { -1,  0,  0, 0 }
                                                     , {  0,  0, -1, 1 }
                                                     , {  0,  0,  0, 1 } } ) );
    }

    SECTION( "Position - rotated beta + gamma" ) {
        SECTION( "degrees" ) {
            auto um = UniversalModule( 0, 0_deg, Angle::deg( -90 ), 90_deg );
            // A part
            CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
            // B part
            // -X
            CHECK( equals( um.getComponentRelativePosition( 3 ), rotate( M_PI_2, { 0, 0, 1 } )
                                                        * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )
                                                        * rotate( - M_PI_2, { 1, 0, 0 } ) ) );
            // +X
            CHECK( equals( um.getComponentRelativePosition( 4 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )  * rotate( - M_PI_2, { 1, 0, 0 } )
                                                        * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // -Z
            CHECK( equals( um.getComponentRelativePosition( 5 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )  * rotate( - M_PI_2, { 1, 0, 0 } )
                                                        * rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // Body
            CHECK( equals( um.getComponentRelativePosition( 8 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } ) ) );
            // Shoe
            CHECK( equals( um.getComponentRelativePosition( 9 ), { {  0, 0, -1, 0 }
                                                        , { -1, 0,  0, 0 }
                                                        , {  0, 1,  0, 1 }
                                                        , {  0, 0,  0, 1 } } ) );
        }

        SECTION( "radians" ) {
            auto um = UniversalModule( 0, 0_rad, Angle::rad( - Angle::pi / 2 ), Angle::rad( Angle::pi / 2 ) );
            // A part
            CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
            // B part
            // -X
            CHECK( equals( um.getComponentRelativePosition( 3 ), rotate( M_PI_2, { 0, 0, 1 } )
                                                        * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )
                                                        * rotate( - M_PI_2, { 1, 0, 0 } ) ) );
            // +X
            CHECK( equals( um.getComponentRelativePosition( 4 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )  * rotate( - M_PI_2, { 1, 0, 0 } )
                                                        * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // -Z
            CHECK( equals( um.getComponentRelativePosition( 5 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )  * rotate( - M_PI_2, { 1, 0, 0 } )
                                                        * rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // Body
            CHECK( equals( um.getComponentRelativePosition( 8 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } ) ) );
            // Shoe
            CHECK( equals( um.getComponentRelativePosition( 9 ), { {  0, 0, -1, 0 }
                                                        , { -1, 0,  0, 0 }
                                                        , {  0, 1,  0, 1 }
                                                        , {  0, 0,  0, 1 } } ) );
        }
    }

    SECTION( "Connector translations" ) {
        CHECK( UniversalModule::translateComponent( "A-X" ) == 0 );
        CHECK( UniversalModule::translateComponent( "A+X" ) == 1 );
        CHECK( UniversalModule::translateComponent( "A-Z" ) == 2 );
        CHECK( UniversalModule::translateComponent( "B-X" ) == 3 );
        CHECK( UniversalModule::translateComponent( "B+X" ) == 4 );
        CHECK( UniversalModule::translateComponent( "B-Z" ) == 5 );
    }
}

TEST_CASE( "Two modules next to each other" ) {
    ModuleId idCounter = 0;
    RofiWorld world;
    auto& m1 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto con = m2.connectors()[ 1 ];
    connect( m1.connectors()[ 0 ], con, Orientation::South );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    REQUIRE( world.prepare() );

    SECTION( "The second is just moved to left by one" ) {
        Matrix new_origin = identity * translate( { -1, 0, 0 } );
        REQUIRE( !equals( identity, new_origin ) );
        Matrix mat = identity;
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 0 ) )
                     , center( new_origin ) ) );

        mat = new_origin * rotate( M_PI, { 0, 0, 1 } );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 1 ) ), center( mat ) ) );

        mat = new_origin * rotate( - M_PI_2, { 0, 1, 0 } );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 2 ) ), center( mat ) ) );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 6 ) ), center( new_origin ) ) );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 7 ) ), center( new_origin ) ) );

        mat =  new_origin * m1.getComponentRelativePosition( 3 );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 3 ) ), center( mat ) ) );

        mat = new_origin * m1.getComponentRelativePosition( 4 );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 4 ) ), center( mat ) ) );

        mat = new_origin * m1.getComponentRelativePosition( 5 );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 5 ) ), center( mat ) ) );

        mat = new_origin * m1.getComponentRelativePosition( 8 );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 8 ) ), center( mat ) ) );

        mat = { { -1, 0, 0, -1 }, { 0, 1, 0, 0 }, { 0, 0, -1, 1 }, { 0, 0, 0, 1 } };
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 9 ) ), center( mat ) ) );

        mat = new_origin * m1.getComponentRelativePosition( 9 );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 9 ) ), center( mat ) ) );
    }
}

TEST_CASE( "Two modules - different angles" ) {
    int idCounter = 0;
    RofiWorld world;
    auto& m1 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    connect( m1.connectors()[ 3 ], m2.connectors()[ 0 ], Orientation::South );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    REQUIRE( world.prepare() );

    SECTION( "BodyA " ) {
        CHECK( equals( world.getModulePosition( m1.getId() ), identity ) );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) ), center( identity * translate( { 1, 0, 1 } ) ) ) );
    }

    SECTION( "BodyB" ) {
    Matrix m1shoeB = m1.getComponentRelativePosition( 9 );
    Matrix m2shoeB = world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 9 );
    CHECK( equals( m1shoeB, { { -1, 0,  0, 0 }
                            , {  0, 1,  0, 0 }
                            , {  0, 0, -1, 1 }
                            , {  0, 0,  0, 1 } } ) );
    CHECK( equals( center( m2shoeB ), center( translate( { 1, 0, 2 } ) ) ) );
    }
}

TEST_CASE( "Three modules -- connect docks 3 to 0s " ) {
    int idCounter = 0;
    RofiWorld world;
    auto& m1 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m3 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    connect( m1.connectors()[ 3 ], m2.connectors()[ 0 ], Orientation::South );
    connect( m2.connectors()[ 3 ], m3.connectors()[ 0 ], Orientation::South );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    REQUIRE( world.prepare() );

    SECTION( "Modules are well placed" ) {
        CHECK( equals( center( world.getModulePosition( m1.getId() ) ), center( identity ) ) );
        CHECK( equals( center( world.getModulePosition( m2.getId() ) ), center( translate( { 1, 0, 1 } ) ) ) );
        CHECK( equals( center( world.getModulePosition( m3.getId() ) ), center( translate( { 2, 0, 2 } ) ) ) );
    }

    SECTION( "Shoes A" ) {
        CHECK( equals( world.getModulePosition( m1.getId() ) * m1.getComponentRelativePosition( 6 ), identity ) );
        CHECK( equals( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 6 ), translate( { 1, 0, 1 } ) ) );
        CHECK( equals( world.getModulePosition( m3.getId() ) * m3.getComponentRelativePosition( 6 ), translate( { 2, 0, 2 } ) ) );
    }

    SECTION( "Shoes B" ) {
        Matrix x = world.getModulePosition( m1.getId() ) * m1.getComponentRelativePosition( 9 );
        CHECK( equals( world.getModulePosition( m1.getId() ) * m1.getComponentRelativePosition( 9 )
                     , translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 1, 0 } ) ) );
        CHECK( equals( world.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 9 )
                     , translate( { 1, 0, 2 } ) * rotate( M_PI, { 0, 1, 0 } ) ) );
        CHECK( equals( world.getModulePosition( m3.getId() ) * m3.getComponentRelativePosition( 9 )
                     , translate( { 2, 0, 3 } ) * rotate( M_PI, { 0, 1, 0 } ) ) );
    }
}

TEST_CASE( "Basic RofiWorld manipulation" ) {
    using namespace rofi;
    int idCounter = 0;
    RofiWorld world;
    auto& m1 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    CHECK( m1.getId() == 0 );
    CHECK( world.getModule( 0 )->getId() == 0 );
    REQUIRE( m1.getId() == 0 );
    REQUIRE( m1.parent == &world );
    CHECK( &m1 == world.getModule( 0 ) );
    auto& m2 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    REQUIRE( m2.getId() == 1 );
    CHECK( m1.getId() == 0 );
    CHECK( world.getModule( 0 )->getId() == 0 );
    REQUIRE( &m1 == world.getModule( 0 ) );
    auto& m3 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    REQUIRE( m3.getId() == 2 );
    REQUIRE( m1.parent == &world );
    auto& m4 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    REQUIRE( m4.getId() == 3 );
    auto& m5 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    REQUIRE( m5.getId() == 4 );
    CHECK( world.modules().size() == 5 );

    CHECK( world.roficomConnections().size() == 0 );
    connect( m1.connectors()[ 5 ], m2.connectors()[ 2 ], Orientation::North );
    connect( m2.connectors()[ 5 ], m3.connectors()[ 2 ], Orientation::North );
    connect( m3.connectors()[ 5 ], m4.connectors()[ 2 ], Orientation::North );
    connect( m4.connectors()[ 5 ], m5.connectors()[ 2 ], Orientation::North );
    CHECK( world.roficomConnections().size() == 4 );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    m1.setGamma( Angle::deg( 90 ) );
    CHECK_FALSE( world.isValid() ); // because the configuration is not prepared
    CHECK( world.validate() );
}

TEST_CASE( "Colliding configuration" ) {
    using namespace rofi;
    int idCounter = 0;
    RofiWorld world;
    auto& m1 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m3 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m4 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m5 = world.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    CHECK( world.modules().size() == 5 );

    connect( m1.connectors()[ 1 ], m2.connectors()[ 2 ], Orientation::North );
    connect( m2.connectors()[ 1 ], m3.connectors()[ 2 ], Orientation::North );
    connect( m3.connectors()[ 1 ], m4.connectors()[ 2 ], Orientation::North );
    connect( m4.connectors()[ 1 ], m5.connectors()[ 2 ], Orientation::North );
    CHECK( world.roficomConnections().size() == 4 );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    CHECK_FALSE( world.validate() );
}

TEST_CASE( "Changing modules ID" ) {
    using namespace rofi;
    RofiWorld world;

    auto& m1 = world.insert( UniversalModule( 0, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = world.insert( UniversalModule( 1, 0_deg, 0_deg, 0_deg ) );
    auto& m3 = world.insert( UniversalModule( 2, 0_deg, 0_deg, 0_deg ) );
    connect( m1.connectors()[ 5 ], m2.connectors()[ 2 ], Orientation::North );
    connect( m2.connectors()[ 5 ], m3.connectors()[ 2 ], Orientation::North );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );

    CHECK( world.validate() );

    CHECK( m1.getId() == 0 );
    CHECK( m2.getId() == 1 );
    CHECK( m3.getId() == 2 );

    CHECK( m2.setId( 42 ) );
    CHECK( world.validate() );
    CHECK( m1.getId() == 0 );
    CHECK( m2.getId() != 1 );
    CHECK( m2.getId() == 42 );
    CHECK( m3.getId() == 2 );

    CHECK( !m2.setId( 0 ) );
    CHECK( m1.getId() == 0 );
    CHECK( m2.getId() == 42 );
    CHECK( m3.getId() == 2 );

    CHECK( m1.setId( 66 ) );
    CHECK( m3.setId( 78 ) );

    CHECK( m1.getId() == 66 );
    CHECK( m2.getId() == 42 );
    CHECK( m3.getId() == 78 );
}

TEST_CASE( "Configurable joints" ) {
    SECTION( "default" ) {
        auto m = UniversalModule( 42, 0_deg, 0_rad, 0_deg );
        int i = 0;
        for ( auto& j : m.configurableJoints() ) {
            static_assert( std::is_same_v< decltype( j ), Joint & > );
            i++;
        }
        REQUIRE( i == 3 );
    }
    SECTION( "const" ) {
        auto m = UniversalModule( 42, 0_deg, 0_rad, 0_deg );
        int i = 0;
        for ( auto& j : std::as_const( m ).configurableJoints() ) {
            static_assert( std::is_same_v< decltype( j ), const Joint & > );
            i++;
        }
        CHECK( i == 3 );
    }
    SECTION( "equality" ) {
        auto m = UniversalModule( 42, 0_deg, 0_rad, 0_deg );

        auto joints = m.configurableJoints();
        auto cjoints = std::as_const( m ).configurableJoints();

        auto it = joints.begin();
        auto cit = cjoints.begin();
        while ( it != joints.end() && cit != cjoints.end() ) {
            CHECK( &*it == &*cit ); // Check address
            ++it;
            ++cit;
        }
        CHECK( it == joints.end() );
        CHECK( cit == cjoints.end() );
    }
}

TEST_CASE( "Connect and disconnect" ) {
    RofiWorld world;

    SECTION( "Disconnect roficomConnections disconnects two modules" ) {
        auto& m1 = world.insert( UniversalModule( 42, 0_deg, 0_deg, 0_deg ) );
        auto& m2 = world.insert( UniversalModule( 66, 0_deg, 0_deg, 0_deg ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        CHECK( world.roficomConnections().empty() );
        auto j = connect( m1.getConnector( "A+X" ), m2.getConnector( "B-Z" ), roficom::Orientation::North );
        CHECK( world.roficomConnections().size() == 1 );
        REQUIRE( world.prepare() );
        CHECK( world.isPrepared() );
        world.disconnect( j );
        CHECK( !world.isPrepared() );
        CHECK( world.roficomConnections().empty() );
    }

    SECTION( "Disconnect on spaceJoint" ) {
        auto& m = world.insert( UniversalModule( 42, 0_deg, 0_deg, 0_deg ) );

        CHECK( world.referencePoints().empty() );
        auto h = connect< RigidJoint >( m.getConnector( "A-Z" ), { 0, 0, 0 }, identity );
        CHECK( !world.referencePoints().empty() );

        REQUIRE( world.prepare() );
        CHECK( world.isPrepared() );
        world.disconnect( h );
        CHECK( !world.isPrepared() );
        CHECK( world.referencePoints().empty() );
    }

    SECTION( "Reconnect - simple" )
    {
        auto & m1 = static_cast< UniversalModule & >(
                world.insert( UniversalModule( 42, 90_deg, 0_deg, 0_deg ) ) );
        auto & m2 = static_cast< UniversalModule & >(
                world.insert( UniversalModule( 66, 0_deg, 90_deg, 0_deg ) ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        auto j1 = connect( m1.getConnector( "A+X" ),
                           m2.getConnector( "A-X" ),
                           roficom::Orientation::South );

        REQUIRE( world.prepare() );
        REQUIRE( world.isPrepared() );
        CHECK( world.isValid() );

        world.disconnect( j1 );

        CHECK_FALSE( world.prepare() );
    }

    SECTION( "Reconnect" )
    {
        auto & m1 = static_cast< UniversalModule & >(
                world.insert( UniversalModule( 42, 90_deg, 0_deg, 0_deg ) ) );
        auto & m2 = static_cast< UniversalModule & >(
                world.insert( UniversalModule( 66, 0_deg, 90_deg, 0_deg ) ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        auto j1 = connect( m1.getConnector( "A+X" ),
                           m2.getConnector( "A-X" ),
                           roficom::Orientation::South );

        REQUIRE( world.prepare() );
        REQUIRE( world.isPrepared() );
        CHECK( world.isValid() );

        m1.setAlpha( 0_deg );
        m2.setBeta( 0_deg );
        auto j2 = connect( m1.getConnector( "B-X" ),
                           m2.getConnector( "B+X" ),
                           roficom::Orientation::South );

        REQUIRE( world.prepare() );
        REQUIRE( world.isPrepared() );
        CHECK( world.isValid() );

        world.disconnect( j1 );

        REQUIRE( world.prepare() );
        REQUIRE( world.isPrepared() );
        CHECK( world.isValid() );

        m1.setAlpha( 90_deg );
        m2.setBeta( 90_deg );

        REQUIRE( world.prepare() );
        REQUIRE( world.isPrepared() );
        CHECK( world.isValid() );
    }
}

TEST_CASE( "Get near connector" ) {
    RofiWorld world;

    SECTION( "two straight modules" ) {
        auto& m1 = world.insert( UniversalModule( 42, 0_deg, 0_deg, 0_deg ) );
        auto& m2 = world.insert( UniversalModule( 66, 0_deg, 0_deg, 0_deg ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        connect( m1.getConnector( "A+X" ), m2.getConnector( "A+X" ), roficom::Orientation::North );

        REQUIRE( world.prepare() );
        CHECK( world.isValid() );

        SECTION( "Works with fixed connection" ) {
            connect( m1.getConnector( "B-X" ), m2.getConnector( "B-X" ), roficom::Orientation::North );

            REQUIRE( world.prepare() );
            CHECK( world.isValid() );
        }

        SECTION( "Get near connector" ) {
            auto nearConnector = m1.getConnector( "B-X" ).getNearConnector();
            REQUIRE( nearConnector.has_value() );
            CHECK( nearConnector->first == m2.getConnector( "B-X" ) );
            CHECK( nearConnector->second == roficom::Orientation::North );

            connect( m1.getConnector( "B-X" ), nearConnector->first, nearConnector->second );

            REQUIRE( world.prepare() );
            CHECK( world.isValid() );
        }
    }

    SECTION( "two straight modules - throws if not prepared in advance" ) {
        auto& m1 = world.insert( UniversalModule( 42, 0_deg, 0_deg, 0_deg ) );
        auto& m2 = world.insert( UniversalModule( 66, 0_deg, 0_deg, 0_deg ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        connect( m1.getConnector( "A+X" ), m2.getConnector( "A+X" ), roficom::Orientation::North );

        REQUIRE_FALSE( world.isPrepared() );

        CHECK_THROWS( m1.getConnector( "B-X" ).getNearConnector() );
    }

    SECTION("universal module with pad") {
        auto um12 = world.insert( UniversalModule( 12, 90_deg, 90_deg, 0_deg ) );
        auto pad42 = world.insert( Pad( 42, 6, 3 ) );

        connect< RigidJoint >( pad42.components().front(),
                                        matrices::Vector(),
                                        matrices::identity );
        connect( um12.getConnector( "A-Z" ),
                    pad42.connectors()[ 1 ],
                    roficom::Orientation::North );

        REQUIRE( world.prepare() );
        REQUIRE( world.isValid() );

        auto connectorB = um12.getConnector( "B-Z" );
        auto nearConnector = connectorB.getNearConnector();
        REQUIRE( nearConnector );
        CHECK( nearConnector->first.parent->getId() == 42 );
        CHECK( nearConnector->first.getIndexInParent() == 4 );
        CHECK( nearConnector->second == roficom::Orientation::South );
    }
}

TEST_CASE( "Configuration copying" ) {
    auto world = RofiWorld();

    SECTION( "RofiWorld copy" ) {
        auto& um12 = world.insert( UniversalModule( 12, 10_deg, 0_deg, 0_deg ) );
        um12.setBeta( 40_deg );

        CHECK( um12.getConnector( "A-Z" ).parent == &um12 );

        CHECK( um12.getAlpha() == 10_deg );
        CHECK( um12.getBeta() == 40_deg );
        CHECK( um12.getGamma() == 0_deg );

        auto world_2 = world;

        auto* um12_2 = dynamic_cast< UniversalModule* >( world_2.getModule( 12 ) );
        REQUIRE( um12_2 );

        CAPTURE( &um12 );
        CHECK( um12_2->getConnector( "A-Z" ).parent == um12_2 );

        CHECK( um12_2->getAlpha().deg() == 10 );
        CHECK( um12_2->getBeta().deg() == 40 );
        CHECK( um12_2->getGamma().deg() == 0 );

        um12_2->setGamma( 60_deg );

        CHECK( um12_2->getGamma() == 60_deg );

        CHECK( um12.getGamma() == 0_deg );

        CHECK( um12.parent == &world );
        CHECK( um12_2->parent == &world_2 );
    }
    SECTION( "Universal Module copy" ) {
        auto um12 = UniversalModule( 12 );
        auto um13copy = um12;
        um13copy.setId( 13 );

        auto& um12_ = world.insert( um12 );
        auto& um13copy_ = world.insert( um13copy );

        CAPTURE( &um12 );
        CAPTURE( &um12_ );
        CAPTURE( &um13copy );
        CAPTURE( &um13copy_ );

        CHECK( um12.getConnector( "B-Z" ).parent == &um12 );
        CHECK( um12_.getConnector( "B-Z" ).parent == &um12_ );
        // CHECK( um13copy.getConnector( "B-Z" ).parent == &um13copy ); // TODO Issue #199
        CHECK( um13copy_.getConnector( "B-Z" ).parent == &um13copy_ );
    }
}

} // namespace