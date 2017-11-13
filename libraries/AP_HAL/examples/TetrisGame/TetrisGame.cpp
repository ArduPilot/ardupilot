//
// Tetris Game APM v0.0.1
//
// Tetris console game example for APM
// Author: Hiroshi Takey, November 2017
//

/**
*   Description.- Tetris game example displaying ASCII render
*   to console, it read input control from console and
*   from RC Channel for move, block change and block speed.
*
*   Keyboard: 'a' = move left, 'd' = move right, ' ' = block change,
*   's' = block speed
*
*   Joy RC: RC Roll = move left and right, RC Pitch UP = block change,
*   RC Pitch Down = block speed
*/

#include "TetrisGame.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

GameTetris gametetris;

Tetris::tetris_block blocks[] = {
    {{"##",    "##"},       2, 2},
    {{" X ",   "XXX"},      3, 2},
    {{"@@@@"},              4, 1},
    {{"OO",    "O ", "O "}, 2, 3},
    {{"&&",    " &", " &"}, 2, 3},
    {{"ZZ ",   " ZZ"},      3, 2}
};

#define TETRIS_PIECES (sizeof(blocks)/sizeof(tetris_block))

#define MAX_CHANNELS    4
// Move block left right
#define JOY_LEFTRIGHT   0
// Change block and speed
#define JOY_CHGDOWN     2

static uint8_t max_channels = 0;
static uint16_t last_value[MAX_CHANNELS];

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(GameTetris, &gametetris, func, _interval_ticks, _max_time_micros)

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task GameTetris::scheduler_tasks[] = {
    SCHED_TASK(ins_update,              50,   1000),
    SCHED_TASK(game_update,              5,   1000),
    SCHED_TASK(proc_cmd,                 5,   1800),
    SCHED_TASK(gameover_chck,           10,   1000),
    SCHED_TASK(rc_update,               10,   1000),
};

void GameTetris::rc_update(void)
{
    bool changed = false;
    uint8_t nchannels = hal.rcin->num_channels();

    if (nchannels > MAX_CHANNELS) {
        nchannels = MAX_CHANNELS;
    }

    for (uint8_t i=0; i < nchannels; i++) {
        uint16_t v = hal.rcin->read(i);
        if (last_value[i] != v) {
            changed = true;
            last_value[i] = v;
        }
        if (i > max_channels) {
            max_channels = i;
        }
    }

    cmd = 0x00;
    if (changed) {
        for (uint8_t i=0; i < max_channels; i++) {
            if (i == JOY_LEFTRIGHT) {
                if (last_value[i] > 1600) {
                    cmd = 'd';
                }
                if (last_value[i] < 1400) {
                    cmd = 'a';
                }
            }

            if (i == JOY_CHGDOWN) {
                if (last_value[i] > 1600) {
                    cmd = ' ';
                }
                if (last_value[i] < 1400) {
                    cmd = 's';
                }
            }
        }
    }
}

void GameTetris::ins_update(void)
{
    ins.update();
}

void GameTetris::game_update(void)
{
    if (!tetris.gameover) {
        count++;
        if ((count % 1) == 0) {
            tetris.tetris_print(&tetris);
        }
        if ((count % 7) == 0) {
            tetris.tetris_gravity(&tetris);
            tetris.tetris_check_lines(&tetris);
        }

        if (!(hal.console->available() <= 0)) {
            cmd = hal.console->read();
        }
    }
}

void GameTetris::gameover_chck(void)
{
    static bool terminate = false;
    if (!terminate && tetris.gameover) {
        terminate = true;
        tetris.tetris_print(&tetris);
        hal.console->printf("*** GAME OVER ***\n");
        tetris.tetris_clean(&tetris);
    }
}

void GameTetris::proc_cmd(void)
{
    switch (cmd) {
        case 'a':
            tetris.x--;
            if (tetris.tetris_hittest(&tetris))
                tetris.x++;
            break;
        case 'd':
            tetris.x++;
            if (tetris.tetris_hittest(&tetris))
                tetris.x--;
            break;
        case 's':
            tetris.tetris_gravity(&tetris);
            break;
        case ' ':
            tetris.tetris_rotate(&tetris);
            break;
    }
    cmd = 0x00;
}

void Tetris::tetris_init(Tetris *tetris,int _w,int _h)
{
    int y_aux;
    tetris->level = 1;
    tetris->score = 0;
    tetris->gameover = 0;
    tetris->w = _w;
    tetris->h = _h;
    size_t sizecnt = sizeof(char*) * _w;
    tetris->game = (char**)malloc(sizecnt);
    sizecnt = sizeof(char) * _h;

    for (int x_aux = 0; x_aux < _w; x_aux++) {
        tetris->game[x_aux] = (char*)malloc(sizecnt);
        for (y_aux = 0; y_aux < _h; y_aux++) {
            tetris->game[x_aux][y_aux] = ' ';
        }
    }
}

void Tetris::tetris_clean(Tetris *tetris)
{
    for (int x_aux = 0; x_aux < tetris->w; x_aux++) {
        free(tetris->game[x_aux]);
    }

    free(tetris->game);
}

void Tetris::tetris_print(Tetris *tetris)
{
    int y_aux;

    for (int x_aux=0; x_aux < tetris->h; x_aux++) {
        hal.console->printf("\n");
    }

    hal.console->printf("[LEVEL: %d | SCORE: %d]\n", tetris->level, tetris->score);

    for (int x_aux=0; x_aux < 2*tetris->w+2; x_aux++)
    {
        hal.console->printf("-");
    }

    hal.console->printf("\n");

    for (y_aux=0; y_aux < tetris->h; y_aux++) {
        hal.console->printf("|");
        for (int x_aux = 0; x_aux < tetris->w; x_aux++) {
            if (x_aux >= tetris->x && y_aux >= tetris->y
                    && x_aux < (tetris->x + tetris->current.w) && y_aux < (tetris->y + tetris->current.h)
                    && tetris->current.data[y_aux - tetris->y][x_aux - tetris->x] != ' ') {
                hal.console->printf("%c ", tetris->current.data[y_aux - tetris->y][x_aux - tetris->x]);
            } else {
                hal.console->printf("%c ", tetris->game[x_aux][y_aux]);
            }
        }

        hal.console->printf("|\n");
    }

    for (int x_aux = 0; x_aux < 2 * tetris->w + 2; x_aux++) {
        hal.console->printf("-");
    }
 
    hal.console->printf("\n");
}

int Tetris::tetris_hittest(Tetris *tetris)
{
    int y_aux, X_aux, Y_aux;
    tetris_block b = tetris->current;

    for (int x_aux = 0; x_aux < b.w; x_aux++) {
        for (y_aux = 0; y_aux < b.h; y_aux++) {
            X_aux=tetris->x + x_aux;
            Y_aux=tetris->y + y_aux;

            if (X_aux < 0 || X_aux >= tetris->w) {
                return 1;
            }

            if (b.data[y_aux][x_aux] != ' ') {
                if ((Y_aux >= tetris->h) ||
                        (X_aux >= 0 && X_aux < tetris->w && Y_aux >= 0 && tetris->game[X_aux][Y_aux] != ' ')) {
                    return 1;
                }
            }
        }
    }

    return 0;
}

void Tetris::tetris_new_block(Tetris *tetris)
{
    tetris->current = blocks[rand() % TETRIS_PIECES];
    tetris->x = (tetris->w / 2) - (tetris->current.w / 2);
    tetris->y = 0;

    if (tetris_hittest(tetris)) {
        tetris->gameover = 1;
    }
}

void Tetris::tetris_print_block(Tetris *tetris)
{
    int y_aux;
    tetris_block b = tetris->current;

    for (int x_aux = 0; x_aux < b.w; x_aux++)
        for (y_aux = 0; y_aux < b.h; y_aux++) {
            if (b.data[y_aux][x_aux] != ' ') {
                tetris->game[tetris->x + x_aux][tetris->y + y_aux] = b.data[y_aux][x_aux];
            }
        }
}

void Tetris::tetris_rotate(Tetris *tetris)
{
    tetris_block b = tetris->current;
    tetris_block s = b;
    int x_aux, y_aux;
    b.w = s.h;
    b.h = s.w;

    for (x_aux = 0; x_aux < s.w; x_aux++) {
        for (y_aux = 0; y_aux < s.h; y_aux++) {
            b.data[x_aux][y_aux] = s.data[s.h-y_aux-1][x_aux];
        }
    }

    x_aux = tetris->x;
    y_aux = tetris->y;
    tetris->x -= (b.w-s.w) / 2;
    tetris->y -= (b.h-s.h) / 2;
    tetris->current = b;

    if (tetris_hittest(tetris)) {
        tetris->current = s;
        tetris->x = x_aux;
        tetris->y = y_aux;
    }
}

void Tetris::tetris_gravity(Tetris *tetris)
{
    tetris->y++;
    if (tetris_hittest(tetris)) {
        tetris->y--;
        tetris_print_block(tetris);
        tetris_new_block(tetris);
    }
}

void Tetris::tetris_fall(Tetris *tetris, int l)
{
    int y_aux;
    for (y_aux = l; y_aux > 0; y_aux--) {
        for (int x_aux = 0; x_aux < tetris->w; x_aux++) {
            tetris->game[x_aux][y_aux] = tetris->game[x_aux][y_aux-1];
        }
    }
    for (int x_aux = 0; x_aux<tetris->w; x_aux++) {
        tetris->game[x_aux][0] = ' ';
    }
}

void Tetris::tetris_check_lines(Tetris *tetris)
{
    int y_aux, l_aux;
    int p = 100;

    for (y_aux = tetris->h-1; y_aux >= 0; y_aux--) {
        l_aux = 1;
        for (int x_aux = 0; x_aux < tetris->w && l_aux; x_aux++) {
            if (tetris->game[x_aux][y_aux] == ' ') {
                l_aux = 0;
            }
        }

        if (l_aux) {
            tetris->score += p;
            p *= 2;
            tetris_fall(tetris, y_aux);
            y_aux++;
        }
    }
}

void Tetris::tetris_run(Tetris &tetris, int _w, int _h)
{
    tetris_init(&tetris, _w, _h);
    srand(TETRIS_PIECES);
    tetris_new_block(&tetris);
}

void GameTetris::setup(void)
{
    AP_Param::setup_sketch_defaults();
    load_parameters();

    hal.console->printf("*** Init Tetris! ***\n");

    AP_BoardConfig::create().init();

    ins.init(scheduler.get_loop_rate_hz());

    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    tetris.tetris_run(tetris, 12,15);

}

void GameTetris::loop(void)
{
    // wait for an INS sample
    ins.wait_for_sample();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all tasks that fit in 20ms
    scheduler.run(20000);
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup(void)
{
    gametetris.setup();
}
void loop(void)
{
    gametetris.loop();
}

AP_HAL_MAIN();
