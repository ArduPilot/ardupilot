/*
 *  Example of AP_Mission Library.
 *  DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_DCM.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

class MissionTest {
public:
    void setup();
    void loop();

private:
    AP_InertialSensor ins;
    AP_Baro baro;
    AP_GPS  gps;
    Compass compass;
    AP_AHRS ahrs{};
    GCS_Dummy _gcs;

    // global constants that control how many verify calls must be made for a command before it completes
    uint8_t verify_nav_cmd_iterations_to_complete = 3;
    uint8_t verify_do_cmd_iterations_to_complete = 1;
    uint8_t num_nav_cmd_runs = 0;
    uint8_t num_do_cmd_runs = 0;

    bool start_cmd(const AP_Mission::Mission_Command& cmd);
    bool verify_cmd(const AP_Mission::Mission_Command& cmd);
    void mission_complete(void);
    void run_mission_test();
    void init_mission();
    void init_mission_no_nav_commands();
    void init_mission_endless_loop();
    void init_mission_jump_to_nonnav();
    void init_mission_starts_with_do_commands();
    void init_mission_ends_with_do_commands();
    void init_mission_ends_with_jump_command();
    void print_mission();
    void run_resume_test();
    void run_set_current_cmd_test();
    void run_set_current_cmd_while_stopped_test();
    void run_replace_cmd_test();
    void run_max_cmd_test();

    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&MissionTest::start_cmd, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&MissionTest::verify_cmd, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&MissionTest::mission_complete, void)};
};

static MissionTest missiontest;

// start_cmd - function that is called when new command is started
//      should return true if command is successfully started
bool MissionTest::start_cmd(const AP_Mission::Mission_Command& cmd)
{
    // reset tracking of number of iterations of this command (we simulate all nav commands taking 3 iterations to complete, all do command 1 iteration)
    if (AP_Mission::is_nav_cmd(cmd)) {
        num_nav_cmd_runs = 0;
        hal.console->printf("started cmd #%d id:%d Nav\n",(int)cmd.index,(int)cmd.id);
    }else{
        num_do_cmd_runs = 0;
        hal.console->printf("started cmd #%d id:%d Do\n",(int)cmd.index,(int)cmd.id);
    }

    return true;
}

// verify_mcd - function that is called repeatedly to ensure a command is progressing
//      should return true once command is completed
bool MissionTest::verify_cmd(const AP_Mission::Mission_Command& cmd)
{
    if (AP_Mission::is_nav_cmd(cmd)) {
        num_nav_cmd_runs++;
        if (num_nav_cmd_runs < verify_nav_cmd_iterations_to_complete) {
            hal.console->printf("verified cmd #%d id:%d Nav iteration:%d\n",(int)cmd.index,(int)cmd.id,(int)num_nav_cmd_runs);
            return false;
        }else{
            hal.console->printf("verified cmd #%d id:%d Nav complete!\n",(int)cmd.index,(int)cmd.id);
            return true;
        }
    }else{
        num_do_cmd_runs++;
        if (num_do_cmd_runs < verify_do_cmd_iterations_to_complete) {
            hal.console->printf("verified cmd #%d id:%d Do iteration:%d\n",(int)cmd.index,(int)cmd.id,(int)num_do_cmd_runs);
            return false;
        }else{
            hal.console->printf("verified cmd #%d id:%d Do complete!\n",(int)cmd.index,(int)cmd.id);
            return true;
        }
    }
}

// mission_complete - function that is called once the mission completes
void MissionTest::mission_complete(void)
{
    hal.console->printf("\nMission Complete!\n");
}

// run_mission_test - tests the stop and resume feature
void MissionTest::run_mission_test()
{
    // uncomment one of the init_xxx() commands below to run the test

    init_mission();                   // run simple mission with many nav commands and one do-jump
    //init_mission_no_nav_commands();   // mission should start the first do command but then complete
    //init_mission_endless_loop();      // mission should ignore the jump that causes the endless loop and complete

    // mission with a do-jump to the previous command which is a "do" command
    //      ideally we would execute this previous "do" command the number of times specified in the do-jump command but this is tricky so we ignore the do-jump
    //      mission should run the "do" command once and then complete
    //init_mission_jump_to_nonnav();

    // mission which starts with do comamnds
    //      first command to execute should be the first do command followed by the first nav command
    //      second do command should execute after 1st do command completes
    //      third do command (which is after 1st nav command) should start after 1st nav command completes
    //init_mission_starts_with_do_commands();

    // init_mission_ends_with_do_commands - initialise a mission which ends with do comamnds
    //      a single do command just after nav command will be started but not verified because mission will complete
    //      final do command will not be started
    //init_mission_ends_with_do_commands();

    // init_mission_ends_with_jump_command - initialise a mission which ends with a jump comamnd
    //      mission should complete after the do-jump is executed the appropriate number of times
    //init_mission_ends_with_jump_command();

    // run_resume_test - tests the stop and resume feature
    //      when mission is resumed, active commands should be started again
    //run_resume_test();

    // run_set_current_cmd_test - tests setting the current command while the mission is running
    //run_set_current_cmd_test();

    // run_set_current_cmd_while_stopped_test - tests setting the current command while the mission is stopped
    //      when mission is resumed, the mission should start from the modified current cmd
    //run_set_current_cmd_while_stopped_test();

    // run_replace_cmd_test - tests replacing a command during a mission
    //run_replace_cmd_test();

    // run_max_cmd_test - tests filling the eeprom with commands and then reading them back
    //run_max_cmd_test();

    // print current mission
    print_mission();

    // start mission
    hal.console->printf("\nRunning missions\n");
    mission.start();

    // update mission forever
    while(true) {
        mission.update();
    }
}

// init_mission - initialise the mission to hold something
void MissionTest::init_mission()
{
    AP_Mission::Mission_Command cmd;

    // clear mission
    mission.clear();

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : first waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : second waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        1234567890,
        -1234567890,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : do-jump to first waypoint 3 times
    cmd.id = MAV_CMD_DO_JUMP;
    cmd.content.jump.target = 2;
    cmd.content.jump.num_times = 1;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #5 : RTL
    cmd.id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }
}

// init_mission_no_nav_commands - initialise a mission with no navigation commands
//      mission should ignore the jump that causes the endless loop and complete
void MissionTest::init_mission_no_nav_commands()
{
    AP_Mission::Mission_Command cmd;

    // clear mission
    mission.clear();

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : "do" command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : "do" command
    cmd.id = MAV_CMD_DO_CHANGE_SPEED;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : "do" command
    cmd.id = MAV_CMD_DO_SET_SERVO;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : do-jump to first command 3 times
    cmd.id = MAV_CMD_DO_JUMP;
    cmd.content.jump.target = 1;
    cmd.content.jump.num_times = 1;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }
}

// init_mission_endless_loop - initialise a mission with a do-jump that causes an endless loop
//      mission should start the first do command but then complete
void MissionTest::init_mission_endless_loop()
{
    AP_Mission::Mission_Command cmd;

    // clear mission
    mission.clear();

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : do-jump command to itself
    cmd.id = MAV_CMD_DO_JUMP;
    cmd.content.jump.target = 1;
    cmd.content.jump.num_times = 2;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }
}

// init_mission_jump_to_nonnav - initialise a mission with a do-jump to the previous command which is a "do" command
//      ideally we would execute this previous "do" command the number of times specified in the do-jump command but this is tricky so we ignore the do-jump
//      mission should run the "do" command once and then complete
void MissionTest::init_mission_jump_to_nonnav()
{
    AP_Mission::Mission_Command cmd;

    // clear mission
    mission.clear();

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : do-roi command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : do-jump command to #2
    cmd.id = MAV_CMD_DO_JUMP;
    cmd.content.jump.target = 2;
    cmd.content.jump.num_times = 2;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }
}

// init_mission_starts_with_do_commands - initialise a mission which starts with do comamnds
//      first command to execute should be the first do command followed by the first nav command
//      second do command should execute after 1st do command completes
//      third do command (which is after 1st nav command) should start after 1st nav command completes
void MissionTest::init_mission_starts_with_do_commands()
{
    AP_Mission::Mission_Command cmd;

    // clear mission
    mission.clear();

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : First "do" command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : Second "do" command
    cmd.id = MAV_CMD_DO_CHANGE_SPEED;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : Third "do" command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #5 : waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        33,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }
}

// init_mission_ends_with_do_commands - initialise a mission which ends with do comamnds
//      a single do command just after nav command will be started but not verified because mission will complete
//      final do command will not be started
void MissionTest::init_mission_ends_with_do_commands()
{
    AP_Mission::Mission_Command cmd;

    // clear mission
    mission.clear();

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };

    // Command #2 : "do" command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        33,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : "do" command after last nav command (but not at end of mission)
    cmd.id = MAV_CMD_DO_CHANGE_SPEED;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #5 : "do" command at end of mission
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }
}

// init_mission_ends_with_jump_command - initialise a mission which ends with a jump comamnd
//      mission should complete after the do-jump is executed the appropriate number of times
void MissionTest::init_mission_ends_with_jump_command()
{
    AP_Mission::Mission_Command cmd;

    // clear mission
    mission.clear();

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : "do" command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        33,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : "do" command after last nav command (but not at end of mission)
    cmd.id = MAV_CMD_DO_CHANGE_SPEED;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #5 : "do" command at end of mission
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #6 : do-jump command to #2 two times
    cmd.id = MAV_CMD_DO_JUMP;
    cmd.content.jump.target = 3;
    cmd.content.jump.num_times = 2;
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }
}

// print_mission - print out the entire mission to the console
void MissionTest::print_mission()
{
    AP_Mission::Mission_Command cmd;

    // check for empty mission
    if (mission.num_commands() == 0) {
        hal.console->printf("No Mission!\n");
        return;
    }

    hal.console->printf("Mission: %d commands\n",(int)mission.num_commands());

    // print each command
    for(uint16_t i=0; i<mission.num_commands(); i++) {
        // get next command from eeprom
        mission.read_cmd_from_storage(i,cmd);

        // print command position in list and mavlink id
        hal.console->printf("Cmd#%d mav-id:%d ", (int)cmd.index, (int)cmd.id);

        // print whether nav or do command
        if (AP_Mission::is_nav_cmd(cmd)) {
            hal.console->printf("Nav ");
        }else{
            hal.console->printf("Do ");
        }

        // print command contents
        if (cmd.id == MAV_CMD_DO_JUMP) {
            hal.console->printf("jump-to:%d num_times:%d\n", (int)cmd.content.jump.target, (int)cmd.content.jump.num_times);
        }else{
            hal.console->printf("p1:%d lat:%ld lng:%ld alt:%ld\n",(int)cmd.p1, (long)cmd.content.location.lat, (long)cmd.content.location.lng, (long)cmd.content.location.alt);
        }
    }
    hal.console->printf("--------\n");
}

// run_resume_test - tests the stop and resume feature
//      when mission is resumed, active commands should be started again
void MissionTest::run_resume_test()
{
    AP_Mission::Mission_Command cmd;

    // create a mission

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : first waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : second waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        1234567890,
        -1234567890,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : do command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #5 : RTL
    cmd.id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // print current mission
    print_mission();

    // start mission
    hal.console->printf("\nRunning missions\n");
    mission.start();

    // update the mission for X iterations
    // set condition to "i<5" to catch mission as cmd #1 (Nav) is running - you should see it restart cmd #1
    // set condition to "i<7" to catch mission just after cmd #1 (Nav) has completed - you should see it start cmd #2
    // set condition to "i<11" to catch mission just after cmd #2 (Nav) has completed - you should see it start cmd #3 (Do) and cmd #4 (Nav)
    for(uint8_t i=0; i<11; i++) {
        mission.update();
    }

    // simulate user pausing the mission
    hal.console->printf("Stopping mission\n");
    mission.stop();

    // update the mission for 5 seconds (nothing should happen)
    uint32_t start_time = AP_HAL::millis();
    while(AP_HAL::millis() - start_time < 5000) {
        mission.update();
    }

    // simulate user resuming mission
    hal.console->printf("Resume mission\n");
    mission.resume();

    // update the mission forever
    while(true) {
        mission.update();
    }
}

// run_set_current_cmd_test - tests setting the current command during a mission
void MissionTest::run_set_current_cmd_test()
{
    AP_Mission::Mission_Command cmd;

    // create a mission

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : do command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : first waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : second waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        1234567890,
        -1234567890,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #5 : do command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #6 : RTL
    cmd.id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // print current mission
    print_mission();

    // start mission
    hal.console->printf("\nRunning missions\n");
    mission.start();

    // update the mission for X iterations to let it go to about command 3 or 4
    for(uint8_t i=0; i<11; i++) {
        mission.update();
    }

    // simulate user setting current command to 2
    hal.console->printf("Setting current command to 2\n");
    mission.set_current_cmd(2);

    // update the mission forever
    while(true) {
        mission.update();
    }
}

// run_set_current_cmd_while_stopped_test - tests setting the current command while the mission is stopped
//      when mission is resumed, the mission should start from the modified current cmd
void MissionTest::run_set_current_cmd_while_stopped_test()
{
    AP_Mission::Mission_Command cmd;

    // create a mission

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : do command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : first waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : second waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        1234567890,
        -1234567890,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #5 : do command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #6 : RTL
    cmd.id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // print current mission
    print_mission();

    // start mission
    hal.console->printf("\nRunning missions\n");
    mission.start();

    // update the mission for X iterations
    for(uint8_t i=0; i<11; i++) {
        mission.update();
    }

    // simulate user pausing the mission
    hal.console->printf("Stopping mission\n");
    mission.stop();

    // simulate user setting current command to 2
    hal.console->printf("Setting current command to 2\n");
    mission.set_current_cmd(2);

    // update the mission for 2 seconds (nothing should happen)
    uint32_t start_time = AP_HAL::millis();
    while(AP_HAL::millis() - start_time < 2000) {
        mission.update();
    }

    // simulate user resuming mission
    hal.console->printf("Resume mission\n");
    mission.resume();

    // wait for the mission to complete
    while(mission.state() != AP_Mission::MISSION_COMPLETE) {
        mission.update();
    }

    // pause for two seconds
    start_time = AP_HAL::millis();
    while(AP_HAL::millis() - start_time < 2000) {
        mission.update();
    }

    // simulate user setting current command to 2 now that the mission has completed
    hal.console->printf("Setting current command to 5\n");
    mission.set_current_cmd(5);

    // simulate user resuming mission
    hal.console->printf("Resume mission\n");
    mission.resume();

    // keep running the mission forever
    while(true) {
        mission.update();
    }
}

// run_replace_cmd_test - tests replacing a command during a mission
void MissionTest::run_replace_cmd_test()
{
    AP_Mission::Mission_Command cmd;

    // create a mission

    // Command #0 : home
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #1 : take-off to 10m
    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        10,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #2 : do command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #3 : first waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #4 : second waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = Location{
        1234567890,
        -1234567890,
        22,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // Command #6 : RTL
    cmd.id = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.p1 = 0;
    cmd.content.location = Location{
        0,
        0,
        0,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.add_cmd(cmd)) {
        hal.console->printf("failed to add command\n");
    }

    // print current mission
    print_mission();

    // start mission
    hal.console->printf("\nRunning missions\n");
    mission.start();

    // update the mission for X iterations to let it go to about command 3 or 4
    for(uint8_t i=0; i<9; i++) {
        mission.update();
    }

    // replace command #4 with a do-command
    // Command #4 : do command
    cmd.id = MAV_CMD_DO_SET_ROI;
    cmd.p1 = 0;
    cmd.content.location = Location{
        12345678,
        23456789,
        11,
        Location::AltFrame::ABSOLUTE
    };
    if (!mission.replace_cmd(4, cmd)) {
        hal.console->printf("failed to replace command 4\n");
    }else{
        hal.console->printf("replaced command #4 -------------\n");
        // print current mission
        print_mission();
    }

    // update the mission forever
    while(true) {
        mission.update();
    }
}

// run_max_cmd_test - tests filling the eeprom with commands and then reading them back
void MissionTest::run_max_cmd_test()
{
    AP_Mission::Mission_Command cmd;
    uint16_t num_commands = 0;
    uint16_t i;
    bool failed_to_add = false;
    bool failed_to_read = false;
    bool success = true;

    // test adding many commands until it fails
    while (!failed_to_add) {
        // Command #0 : home
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 0;
        cmd.content.location = Location{
            12345678,
            23456789,
            num_commands,
            Location::AltFrame::ABSOLUTE
        };
        if (!mission.add_cmd(cmd)) {
            hal.console->printf("failed to add command #%u, library says max is %u\n",(unsigned int)num_commands, (unsigned int)mission.num_commands_max());
            failed_to_add = true;
        }else{
            num_commands++;
        }
    }

    // test retrieving commands
    for (i=0; i<num_commands; i++) {
        if (!mission.read_cmd_from_storage(i,cmd)) {
            hal.console->printf("failed to retrieve command #%u\n",(unsigned int)i);
            failed_to_read = true;
            break;
        }else{
            if (cmd.content.location.alt == i) {
                hal.console->printf("successfully read command #%u\n",(unsigned int)i);
            }else{
                hal.console->printf("cmd %u's alt does not match, expected %u but read %u\n",
                                      (unsigned int)i,(unsigned int)i,(unsigned int)cmd.content.location.alt);
            }
        }
    }

    // final success/fail message
    if (num_commands != mission.num_commands()) {
        hal.console->printf("\nTest failed!  Only wrote %u instead of %u commands",(unsigned int)i,(unsigned int)mission.num_commands_max());
        success = false;
    }
    if (failed_to_read) {
        hal.console->printf("\nTest failed!  Only read %u instead of %u commands",(unsigned int)i,(unsigned int)mission.num_commands_max());
        success = false;
    }
    if (success) {
        hal.console->printf("\nTest Passed!  wrote and read back %u commands\n\n",(unsigned int)mission.num_commands_max());
    }
}

// setup
void MissionTest::setup(void)
{
    hal.console->printf("AP_Mission library test\n\n");

    // display basic info about command sizes
    hal.console->printf("Max Num Commands: %d\n",(int)mission.num_commands_max());
    hal.console->printf("Command size: %d bytes\n",(int)AP_MISSION_EEPROM_COMMAND_SIZE);
}

// loop
void MissionTest::loop(void)
{
    // uncomment line below to run one of the mission tests
    run_mission_test();

    // uncomment line below to run the mission pause/resume test
    //run_resume_test();

    // wait forever
    while(true) {
        hal.scheduler->delay(1000);
    }
}

void setup(void);
void loop(void);

void setup(void)
{
    missiontest.setup();
}
void loop(void)
{
    missiontest.loop();
}

AP_HAL_MAIN();
