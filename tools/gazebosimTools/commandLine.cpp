#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include <gazebo/gazebo_client.hh>

#include "rofi_hal.hpp"


enum class JointCmd
{
    GET_CAPABILITIES,
    GET_VELOCITY,
    SET_VELOCITY,
    GET_POSITION,
    SET_POSITION,
    GET_TORQUE,
    SET_TORQUE,
};

enum class ConnectorCmd
{
    CONNECT,
    DISCONNECT,
    GET_STATE,
    PACKET,
    CONNECT_POWER,
    DISCONNECT_POWER,
};

inline std::string_view ltrim( std::string_view line, std::string_view ws = " \t" )
{
    auto first = line.find_first_not_of( ws );
    return first == std::string_view::npos ? "" : line.substr( first );
}

inline std::string_view rtrim( std::string_view line, std::string_view ws = " \t" )
{
    auto last = line.find_last_not_of( ws );
    return last == std::string_view::npos ? "" : line.substr( 0, last + 1 );
}

inline std::string_view trim( std::string_view line, std::string_view ws = " \t" )
{
    return rtrim( ltrim( line, ws ), ws );
}

inline std::vector< std::string_view > split( std::string_view line )
{
    const std::string_view ws = " \t";

    line = trim( line, ws );
    std::vector< std::string_view > tokens;
    while ( !line.empty() && line.front() != '#' )
    {
        auto last = line.find_first_of( ws );
        tokens.push_back( line.substr( 0, last ) );

        auto nextFirst = line.find_first_not_of( ws, last );
        if ( nextFirst == std::string_view::npos )
        {
            break;
        }

        line.remove_prefix( nextFirst );
    }

    return tokens;
}

float readFloat( std::string_view str )
{
    return float( std::atof( std::string( str ).data() ) );
}

int readInt( std::string_view str )
{
    return std::atoi( std::string( str ).data() );
}

std::string toString( rofi::hal::ConnectorPosition position )
{
    using rofi::hal::ConnectorPosition;

    switch ( position )
    {
        case ConnectorPosition::Retracted:
        {
            return "retracted";
        }
        case ConnectorPosition::Extended:
        {
            return "extended";
        }
    }

    throw std::runtime_error( "Unknown connector position" );
}

std::string toString( rofi::hal::ConnectorOrientation orientation )
{
    using rofi::hal::ConnectorOrientation;

    switch ( orientation )
    {
        case ConnectorOrientation::North:
        {
            return "north";
        }
        case ConnectorOrientation::East:
        {
            return "east";
        }
        case ConnectorOrientation::South:
        {
            return "south";
        }
        case ConnectorOrientation::West:
        {
            return "west";
        }
    }

    throw std::runtime_error( "Unknown connector orientation" );
}


JointCmd getJointCmdType( std::string_view token )
{
    const std::map< std::string_view, JointCmd > map = {
        { "gc", JointCmd::GET_CAPABILITIES }, { "getcapabilities", JointCmd::GET_CAPABILITIES },
        { "gp", JointCmd::GET_POSITION },     { "getposition", JointCmd::GET_POSITION },
        { "gv", JointCmd::GET_VELOCITY },     { "getvelocity", JointCmd::GET_VELOCITY },
        { "gt", JointCmd::GET_TORQUE },       { "gettorque", JointCmd::GET_TORQUE },
        { "sp", JointCmd::SET_POSITION },     { "setposition", JointCmd::SET_POSITION },
        { "sv", JointCmd::SET_VELOCITY },     { "setvelocity", JointCmd::SET_VELOCITY },
        { "st", JointCmd::SET_TORQUE },       { "settorque", JointCmd::SET_TORQUE },
    };

    auto it = map.find( token );
    if ( it == map.end() )
        throw std::runtime_error( "Unknown joint cmd type" );
    return it->second;
}

inline ConnectorCmd getConnectorCmdType( std::string_view token )
{
    const std::map< std::string_view, ConnectorCmd > map = {
        { "gs", ConnectorCmd::GET_STATE },
        { "getstate", ConnectorCmd::GET_STATE },
        { "c", ConnectorCmd::CONNECT },
        { "connect", ConnectorCmd::CONNECT },
        { "d", ConnectorCmd::DISCONNECT },
        { "disconnect", ConnectorCmd::DISCONNECT },
        { "sp", ConnectorCmd::PACKET },
        { "sendpacket", ConnectorCmd::PACKET },
        { "cp", ConnectorCmd::CONNECT_POWER },
        { "connectpower", ConnectorCmd::CONNECT_POWER },
        { "dp", ConnectorCmd::DISCONNECT_POWER },
        { "disconnectpower", ConnectorCmd::DISCONNECT_POWER },
    };

    auto it = map.find( token );
    if ( it == map.end() )
        throw std::runtime_error( "Unknown connector cmd type" );
    return it->second;
}

void printHelp()
{
    std::cerr << "\nUsage:\n";
    std::cerr << "\tquit (q)\n";
    std::cerr << "\tdescriptor (d)\n";
    std::cerr << "\tjoint (j) <joint_number> <joint_command>\n";
    std::cerr << "\tconnector (c) <connector_number> <connector_command>\n";

    std::cerr << "\nJoint commands:\n";
    std::cerr << "\tgetcapabilities (gc)\n";
    std::cerr << "\tgetposition (gp)\n";
    std::cerr << "\tgetvelocity (gv)\n";
    std::cerr << "\tgettorque (gt)\n";
    std::cerr << "\tsetposition (sp) <position> <velocity>\n";
    std::cerr << "\tsetvelocity (sv) <velocity>\n";
    std::cerr << "\tsettorque (st) <torque>\n";

    std::cerr << "\nConnector commands:\n";
    std::cerr << "\tgetstate (gs)\n";
    std::cerr << "\tconnect (c)\n";
    std::cerr << "\tdisconnect (d)\n";
    // std::cerr << "\tsendpacket (sp)\n"; // not implemented
    // std::cerr << "\tconnectpower (cp)\n"; // not implemented
    // std::cerr << "\tdisconnectpower (dp)\n"; // not implemented
}

void processJointCmd( rofi::hal::RoFI & rofi, const std::vector< std::string_view > & tokens )
{
    if ( tokens.size() < 3 )
        throw std::runtime_error( "Wrong number of arguments" );

    auto joint = rofi.getJoint( readInt( tokens[ 1 ] ) );

    switch ( getJointCmdType( tokens[ 2 ] ) )
    {
        case JointCmd::GET_CAPABILITIES:
        {
            auto maxPosition = joint.maxPosition();
            auto minPosition = joint.minPosition();
            auto maxSpeed = joint.maxSpeed();
            auto minSpeed = joint.minSpeed();
            auto maxTorque = joint.maxTorque();
            std::cout << "Max position: " << maxPosition << "\n";
            std::cout << "Min position: " << minPosition << "\n";
            std::cout << "Max speed: " << maxSpeed << "\n";
            std::cout << "Min speed: " << minSpeed << "\n";
            std::cout << "Max torque: " << maxTorque << "\n";
            break;
        }
        case JointCmd::GET_VELOCITY:
        {
            auto result = joint.getVelocity();
            std::cout << "Current velocity: " << result << "\n";
            break;
        }
        case JointCmd::SET_VELOCITY:
        {
            if ( tokens.size() != 4 )
            {
                throw std::runtime_error( "Wrong number of arguments" );
            }
            joint.setVelocity( readFloat( tokens[ 3 ] ) );
            break;
        }
        case JointCmd::GET_POSITION:
        {
            auto result = joint.getPosition();
            std::cout << "Current position: " << result << "\n";
            break;
        }
        case JointCmd::SET_POSITION:
        {
            if ( tokens.size() != 5 )
            {
                throw std::runtime_error( "Wrong number of arguments" );
            }
            joint.setPosition( readFloat( tokens[ 3 ] ),
                               readFloat( tokens[ 4 ] ),
                               []( rofi::hal::Joint ) { std::cout << "Position reached\n"; } );
            break;
        }
        case JointCmd::GET_TORQUE:
        {
            auto result = joint.getTorque();
            std::cout << "Current torque: " << result << "\n";
            break;
        }
        case JointCmd::SET_TORQUE:
        {
            if ( tokens.size() != 4 )
            {
                throw std::runtime_error( "Wrong number of arguments" );
            }
            joint.setTorque( readFloat( tokens[ 3 ] ) );
            break;
        }
    }
}

void processConnectorCmd( rofi::hal::RoFI & rofi, const std::vector< std::string_view > & tokens )
{
    if ( tokens.size() < 3 )
    {
        throw std::runtime_error( "Wrong number of arguments" );
    }

    auto connector = rofi.getConnector( readInt( tokens[ 1 ] ) );

    switch ( getConnectorCmdType( tokens[ 2 ] ) )
    {
        case ConnectorCmd::CONNECT:
        {
            connector.connect();
            std::cout << "Connecting\n";
            break;
        }
        case ConnectorCmd::DISCONNECT:
        {
            connector.disconnect();
            std::cout << "Disconnecting\n";
            break;
        }
        case ConnectorCmd::GET_STATE:
        {
            auto result = connector.getState();
            std::cout << "Position: " << toString( result.position ) << "\n";
            std::cout << "Connected: " << std::boolalpha << result.connected << "\n";
            if ( result.connected )
            {
                std::cout << "Orientation: " << toString( result.orientation ) << "\n";
            }
            break;
        }
        case ConnectorCmd::PACKET:
        case ConnectorCmd::CONNECT_POWER:
        case ConnectorCmd::DISCONNECT_POWER:
        {
            std::cout << "Connector command not implemented\n";
            printHelp();
            break;
        }
    }
}

rofi::hal::RoFI getRoFI( std::optional< rofi::hal::RoFI::Id > id )
{
    if ( id )
    {
        return rofi::hal::RoFI::getRemoteRoFI( *id );
    }
    return rofi::hal::RoFI::getLocalRoFI();
}


int main( int argc, char ** argv )
{
    try
    {
        std::optional< rofi::hal::RoFI::Id > rofiId;

        if ( argc > 1 )
        {
            rofiId = readInt( argv[ 1 ] );
        }

        if ( rofiId )
        {
            std::cerr << "Connecting to RoFI " << *rofiId << "\n";
        }
        else
        {
            std::cerr << "Connecting to local RoFI\n";
        }

        auto rofi = getRoFI( rofiId );
        std::cerr << "Connected to RoFI " << rofi.getId() << "\n";

        for ( std::string line; std::getline( std::cin, line ); )
        {
            try
            {
                auto tokens = split( line );
                if ( tokens.empty() )
                {
                    continue;
                }

                if ( tokens.front() == "q" || tokens.front() == "quit" )
                {
                    break;
                }

                if ( tokens.front() == "d" || tokens.front() == "descriptor" )
                {
                    auto descriptor = rofi.getDescriptor();
                    std::cout << "Joint count: " << descriptor.jointCount << "\n";
                    std::cout << "Connector count: " << descriptor.connectorCount << "\n";
                    continue;
                }

                if ( tokens.front() == "j" || tokens.front() == "joint" )
                {
                    processJointCmd( rofi, tokens );
                    continue;
                }

                if ( tokens.front() == "c" || tokens.front() == "connector" )
                {
                    processConnectorCmd( rofi, tokens );
                    continue;
                }

                std::cerr << "Unknown command\n";
                printHelp();
            }
            catch ( const std::runtime_error & e )
            {
                std::cerr << "ERROR: " << e.what() << "\n";
            }
        }
    }
    catch ( const gazebo::common::Exception & e )
    {
        std::cerr << e.GetErrorStr() << "\n";
    }
}
