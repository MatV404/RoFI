#include "simplesim/simplesim.hpp"


namespace rofi::simplesim
{
void Simplesim::run( Simplesim::OnConfigurationUpdate onConfigurationUpdate,
                     std::stop_token stopToken )
{
    assert( onConfigurationUpdate && "Provide on configuration update callback" );
    if ( !onConfigurationUpdate ) {
        onConfigurationUpdate = []( auto && ) {};
    }

    assert( _simulation );
    assert( _communication );
    auto & simulation = *_simulation;
    auto & communication = *_communication;

    assert( simulation.moduleStates() );
    auto lastConfiguration = simulation.moduleStates()->currentConfiguration();

    while ( !stopToken.stop_requested() ) {
        auto startTime = std::chrono::steady_clock::now();

        auto settings = _settings.copy();
        if ( settings.isPaused() ) {
            // TODO reactive waiting
            if ( !_configurationProvided.test_and_set() ) {
                onConfigurationUpdate( lastConfiguration );
            }
            std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
            continue;
        }

        auto [ responses,
               newConfiguration ] = simulation.simulateOneIteration( settings.getSimStepTime() );

        lastConfiguration = std::move( newConfiguration );
        assert( onConfigurationUpdate );
        onConfigurationUpdate( lastConfiguration );
        communication.sendRofiResponses( std::move( responses ) );

        std::this_thread::sleep_until( startTime + settings.getRealStepTime() );
    }
}

// Can be called from any thread
ServerSettings Simplesim::onSettingsCmd( const msgs::SettingsCmd & settingsCmd )
{
    if ( settingsCmd.cmd_type() == msgs::SettingsCmd::SEND_CONFIGURATION_AND_STATE ) {
        _configurationProvided.clear();
    }

    using NewValueCase = msgs::SettingsCmd::NewValueCase;

    return _settings.visit( [ &settingsCmd ]( ServerSettings & settings ) {
        switch ( settingsCmd.cmd_type() ) {
            case msgs::SettingsCmd::SEND_CONFIGURATION_AND_STATE:
            {
                break;
            }

            case msgs::SettingsCmd::PAUSE:
            {
                if ( settingsCmd.new_value_case() != NewValueCase::NEW_VALUE_NOT_SET ) {
                    std::cerr << "Settings cmd to resume has a set value.\n";
                    break;
                }
                settings.pause();
                break;
            }
            case msgs::SettingsCmd::RESUME:
            {
                if ( settingsCmd.new_value_case() != NewValueCase::NEW_VALUE_NOT_SET ) {
                    std::cerr << "Settings cmd to resume has a set value.\n";
                    break;
                }
                settings.resume();
                break;
            }
            case msgs::SettingsCmd::CHANGE_SPEED_RATIO:
            {
                if ( settingsCmd.new_value_case() != NewValueCase::kSimSpeedRatio
                     && settingsCmd.new_value_case() != NewValueCase::NEW_VALUE_NOT_SET )
                {
                    std::cerr << "Set value in settings cmd doesn't match the cmd type.\n"
                              << "Ignoring...\n";
                    break;
                }
                auto simSpeedRatio = settingsCmd.sim_speed_ratio();
                if ( simSpeedRatio < 0.f ) {
                    std::cerr << "Cannot set speed ratio to a negative number (got: "
                              << simSpeedRatio << ")\nIgnoring...\n";
                    break;
                }
                settings.setSimSpeedRatio( simSpeedRatio );
                break;
            }
            case msgs::SettingsCmd::CHANGE_SIM_STEP_TIME:
            {
                if ( settingsCmd.new_value_case() != NewValueCase::kSimStepTimeMs
                     && settingsCmd.new_value_case() != NewValueCase::NEW_VALUE_NOT_SET )
                {
                    std::cerr << "Set value in settings cmd doesn't match the cmd type.\n"
                              << "Ignoring...\n";
                    break;
                }
                auto simStepTimeMs = settingsCmd.sim_step_time_ms();
                if ( simStepTimeMs <= 0 ) {
                    std::cerr << "Cannot set sim step time to a nonpositive number (got: "
                              << simStepTimeMs << ")\nIgnoring...\n";
                    break;
                }
                settings.setSimStepTimeMs( simStepTimeMs );
                break;
            }

            default:
            {
                std::cerr << "Unknown settings cmd type (cmd_type: " << settingsCmd.cmd_type()
                          << ")\nIgnoring...\n";
            }
        }

        return settings;
    } );
}

} // namespace rofi::simplesim
