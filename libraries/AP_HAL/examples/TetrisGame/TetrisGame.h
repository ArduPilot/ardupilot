#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "TetrisGame.h"
#include "Parameters.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

class Tetris {
public:
    Tetris()
    {}

    typedef struct __tetris_block {
        char data[5][5];
        int w;
        int h;
    } tetris_block;

    void tetris_run(Tetris &tetris, int _w, int _h);
    void tetris_init(Tetris *tetris,int _w,int _h);
    void tetris_clean(Tetris *tetris);
    void tetris_print(Tetris *tetris);
    int tetris_hittest(Tetris *tetris);
    void tetris_new_block(Tetris *tetris);
    void tetris_rotate(Tetris *tetris);
    void tetris_gravity(Tetris *tetris);
    void tetris_fall(Tetris *tetris, int l);
    void tetris_check_lines(Tetris *tetris);
    void tetris_print_block(Tetris *tetris);

    int gameover;
    int x;
    int y;

private:

    int level;
    int score;
    int w;
    int h;
    char **game;
    tetris_block current;

};

class GameTetris {
public:
    GameTetris() :
        count(0)
    {}

    void setup();
    void loop();

private:
    AP_Scheduler scheduler = AP_Scheduler::create();
    AP_InertialSensor ins = AP_InertialSensor::create();
 
    Tetris tetris;

    int count;
    char cmd;

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    // setup the var_info table
    AP_Param param_loader{var_info};
    Parameters g;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    void ins_update(void);
    void gameover_chck(void);
    void game_update(void);
    void proc_cmd(void);
    void rc_update(void);

    void load_parameters(void);
};

extern const AP_HAL::HAL& hal;
extern GameTetris gametetris;
