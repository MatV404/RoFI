#pragma once

#include <array>
#include <iostream>
#include <string_view>
#include <configuration/rofiworld.hpp>

namespace rofi::configuration {

class UniversalModule : public Module {
    Angle _getJointAngle( int i ) const {
        return Angle::rad( joints()[ i ].joint->positions()[ 0 ] );
    }

    static std::vector< Component > _initComponents();
    static std::vector< ComponentJoint > _initJoints();

    /**
     * Build an universal module with given angles of the respective joints.
     */
    UniversalModule buildUniversalModule( int id, Angle alpha, Angle beta, Angle gamma );

    enum UmComponents { UmBodyA = 7, UmBodyB = 8, UmShoeA = 6, UmShoeB = 9 };

public:
    ATOMS_CLONEABLE( UniversalModule );

    explicit UniversalModule( int id ) : UniversalModule( id, 0_deg, 0_deg, 0_deg ) {};
    UniversalModule( int id, Angle a, Angle b, Angle g )
    : Module( ModuleType::Universal, _initComponents(), 6, _initJoints(), id ) {
        setAlpha( a ); setBeta( b ); setGamma( g );
    }

    Angle getAlpha() const {
        return _getJointAngle( 0 );
    }
    Angle getBeta()  const {
        return _getJointAngle( 1 );
    }
    Angle getGamma() const {
        return _getJointAngle( 2 );
    }

    void setAlpha( Angle a ) {
        std::array tmp{ a.rad() };
        setJointPositions( 0, tmp );
    }
    void setBeta( Angle a ) {
        std::array tmp{ a.rad() };
        setJointPositions( 1, tmp );
    }
    void setGamma( Angle a ) {
        std::array tmp{ a.rad() };
        setJointPositions( 2, tmp );
    }

    Component getBodyA() {
        return components()[ UmBodyA ];
    }
    const Component getBodyA() const {
        return components()[ UmBodyA ];
    }
    Component getBodyB() {
        return components()[ UmBodyB ];
    }
    const Component getBodyB() const {
        return components()[ UmBodyB ];
    }

    const auto& getConnector( std::string_view cStr ) const {
        return components()[ translateComponent( cStr ) ];
    }

    static int translateComponent( std::string_view cStr );
    static std::string_view translateComponent( int c );
};


/**
 * Given a stream read a configuration from the old "Viki" format. Unlike the
 * legacy parser, supports only a single configuration per file.
 */
RofiWorld readOldConfigurationFormat( std::istream& s );

} // namespace rofi::configuration

