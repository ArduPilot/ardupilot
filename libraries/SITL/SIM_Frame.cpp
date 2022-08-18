/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  multicopter frame simulator class
*/

#include "SIM_Frame.h"
#include <AP_Motors/AP_Motors.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include <stdio.h>
#include <sys/stat.h>


using namespace SITL;

static Motor quad_plus_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
};

static Motor quad_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

// motor order to match betaflight conventions
// See: https://fpvfrenzy.com/betaflight-motor-order/
static Motor quad_bf_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,1),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4),
};

// motor order to match betaflight conventions, reversed direction
static Motor quad_bf_x_rev_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
};

// motor order to match DJI conventions
// See: https://forum44.djicdn.com/data/attachment/forum/201711/26/172348bppvtt1ot1nrtp5j.jpg
static Motor quad_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

// motor order so that test order matches motor order ("clockwise X")
static Motor quad_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
};

static Motor tiltquad_h_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, -1, 0, 0, 7, 10, -90),
};

static Motor hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor hexax_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4)
};

static Motor hexa_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_3,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_6,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2)
};

static Motor hexa_cw_x_motors[] = 
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

static Motor octa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor octa_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8),
    Motor(AP_MOTORS_MOT_3,  -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7),
    Motor(AP_MOTORS_MOT_4, -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_5, -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6,  157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_7,  112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_8,   67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2)
};

static Motor octa_cw_x_motors[] = 
{
    Motor(AP_MOTORS_MOT_1,   22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,   67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3,  112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5, -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6, -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_7,  -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7),
    Motor(AP_MOTORS_MOT_8,  -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8)
};

static Motor octa_quad_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

static Motor octa_quad_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_7,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7),
    Motor(AP_MOTORS_MOT_8,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8)
};

static Motor dodeca_hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3),
    Motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4),
    Motor(AP_MOTORS_MOT_5,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7),
    Motor(AP_MOTORS_MOT_8, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8),
    Motor(AP_MOTORS_MOT_9,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10),
    Motor(AP_MOTORS_MOT_11, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11),
    Motor(AP_MOTORS_MOT_12, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12)
};

static Motor deca_motors[] =
{
    Motor(AP_MOTORS_MOT_1,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,    36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,    72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_4,   108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4),
    Motor(AP_MOTORS_MOT_5,   144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,   180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7,  -144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7),
    Motor(AP_MOTORS_MOT_8,  -108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8),
    Motor(AP_MOTORS_MOT_9,   -72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10,  -36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10)
};

static Motor deca_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    18, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,    54, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_4,   126, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4),
    Motor(AP_MOTORS_MOT_5,   162, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,  -162, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7,  -126, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7),
    Motor(AP_MOTORS_MOT_8,   -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8),
    Motor(AP_MOTORS_MOT_9,   -54, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10,  -18, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10)
};

static Motor tri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2)
};

static Motor y6_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_6, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3)
};

/*
  FireflyY6 is a Y6 with front motors tiltable using servo on channel 9 (output 8)
 */
static Motor firefly_motors[] =
{
    Motor(AP_MOTORS_MOT_1, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_2,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_6, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6, -1, 0, 0, 6, 0, -90)
};

/*
  table of supported frame types. String order is important for
  partial name matching
 */
static Frame supported_frames[] =
{
    Frame("+",         4, quad_plus_motors),
    Frame("quad",      4, quad_plus_motors),
    Frame("copter",    4, quad_plus_motors),
    Frame("x",         4, quad_x_motors),
    Frame("bfxrev",    4, quad_bf_x_rev_motors),
    Frame("bfx",       4, quad_bf_x_motors),
    Frame("djix",      4, quad_dji_x_motors),
    Frame("cwx",       4, quad_cw_x_motors),
    Frame("tilthvec",  4, tiltquad_h_vectored_motors),
    Frame("hexax",     6, hexax_motors),
    Frame("hexa-cwx",  6, hexa_cw_x_motors),
    Frame("hexa-dji",  6, hexa_dji_x_motors),
    Frame("hexa",      6, hexa_motors),
    Frame("octa-cwx",  8, octa_cw_x_motors),
    Frame("octa-dji",  8, octa_dji_x_motors),
    Frame("octa-quad-cwx",8, octa_quad_cw_x_motors),
    Frame("octa-quad", 8, octa_quad_motors),
    Frame("octa",      8, octa_motors),
    Frame("deca",     10, deca_motors),
    Frame("deca-cwx", 10, deca_cw_x_motors),
    Frame("dodeca-hexa", 12, dodeca_hexa_motors),
    Frame("tri",       3, tri_motors),
    Frame("tilttrivec",3, tilttri_vectored_motors),
    Frame("tilttri",   3, tilttri_motors),
    Frame("y6",        6, y6_motors),
    Frame("firefly",   6, firefly_motors)
};

// get air density in kg/m^3
float Frame::get_air_density(float alt_amsl) const
{
    float sigma, delta, theta;

    AP_Baro::SimpleAtmosphere(alt_amsl * 0.001f, sigma, delta, theta);

    const float air_pressure = SSL_AIR_PRESSURE * delta;
    return air_pressure / (ISA_GAS_CONSTANT * (C_TO_KELVIN(model.refTempC)));
}

#if USE_PICOJSON
/*
  load frame specific parameters from a json file if available
 */
void Frame::load_frame_params(const char *model_json)
{
    char *fname = nullptr;
    struct stat st;
    if (AP::FS().stat(model_json, &st) == 0) {
        fname = strdup(model_json);
    } else {
        IGNORE_RETURN(asprintf(&fname, "@ROMFS/models/%s", model_json));
        if (AP::FS().stat(model_json, &st) != 0) {
            AP_HAL::panic("%s failed to load\n", model_json);
        }
    }
    if (fname == nullptr) {
        AP_HAL::panic("%s failed to load\n", model_json);
    }
    ::printf("Loading model %s\n", fname);
    int fd = AP::FS().open(model_json, O_RDONLY);
    if (fd == -1) {
        AP_HAL::panic("%s failed to load\n", model_json);
    }
    char buf[st.st_size+1];
    memset(buf, '\0', sizeof(buf));
    if (AP::FS().read(fd, buf, st.st_size) != st.st_size) {
        AP_HAL::panic("%s failed to load\n", model_json);
    }
    AP::FS().close(fd);

    char *start = strchr(buf, '{');
    if (!start) {
        AP_HAL::panic("Invalid json %s", model_json);
    }
    free(fname);

    /*
      remove comments, as not allowed by the parser
     */
    for (char *p = strchr(start,'#'); p; p=strchr(p+1, '#')) {
        // clear to end of line
        do {
            *p++ = ' ';
        } while (*p != '\n' && *p != '\r' && *p);
    }

    picojson::value obj;
    std::string err = picojson::parse(obj, start);
    if (!err.empty()) {
        AP_HAL::panic("Failed to load %s: %s", model_json, err.c_str());
        exit(1);
    }

    enum class VarType {
        FLOAT,
        VECTOR3F,
    };

    struct json_search {
        const char *label;
        void *ptr;
        VarType t;
    };
    
    json_search vars[] = {
#define FRAME_VAR(s) { #s, &model.s, VarType::FLOAT }
        FRAME_VAR(mass),
        FRAME_VAR(diagonal_size),
        FRAME_VAR(refSpd),
        FRAME_VAR(refAngle),
        FRAME_VAR(refVoltage),
        FRAME_VAR(refCurrent),
        FRAME_VAR(refAlt),
        FRAME_VAR(refTempC),
        FRAME_VAR(maxVoltage),
        FRAME_VAR(battCapacityAh),
        FRAME_VAR(refBatRes),
        FRAME_VAR(propExpo),
        FRAME_VAR(refRotRate),
        FRAME_VAR(hoverThrOut),
        FRAME_VAR(pwmMin),
        FRAME_VAR(pwmMax),
        FRAME_VAR(spin_min),
        FRAME_VAR(spin_max),
        FRAME_VAR(slew_max),
        FRAME_VAR(disc_area),
        FRAME_VAR(mdrag_coef),
        {"moment_inertia", &model.moment_of_inertia, VarType::VECTOR3F},
        FRAME_VAR(num_motors),
    };

    for (uint8_t i=0; i<ARRAY_SIZE(vars); i++) {
        auto v = obj.get(vars[i].label);
        if (v.is<picojson::null>()) {
            // use default value
            continue;
        }
        if (vars[i].t == VarType::FLOAT) {
            parse_float(v, vars[i].label, *((float *)vars[i].ptr));

        } else if (vars[i].t == VarType::VECTOR3F) {
            parse_vector3(v, vars[i].label, *(Vector3f *)vars[i].ptr);

        }
    }

    json_search per_motor_vars[] = {
        {"position", &model.motor_pos, VarType::VECTOR3F},
        {"vector", &model.motor_thrust_vec, VarType::VECTOR3F},
        {"yaw", &model.yaw_factor, VarType::FLOAT},
    };
    char label_name[20];
    for (uint8_t i=0; i<ARRAY_SIZE(per_motor_vars); i++) {
        for (uint8_t j=0; j<12; j++) {
            sprintf(label_name, "motor%i_%s", j+1, per_motor_vars[i].label);
            auto v = obj.get(label_name);
            if (v.is<picojson::null>()) {
                // use default value
                continue;
            }
            if (vars[i].t == VarType::FLOAT) {
                parse_float(v, label_name, *(((float *)per_motor_vars[i].ptr) + j));

            } else if (per_motor_vars[i].t == VarType::VECTOR3F) {
                parse_vector3(v, label_name, *(((Vector3f *)per_motor_vars[i].ptr) + j));
            }
        }
    }

    ::printf("Loaded model params from %s\n", model_json);
}

void Frame::parse_float(picojson::value val, const char* label, float &param) {
    if (!val.is<double>()) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    param = val.get<double>();
}

void Frame::parse_vector3(picojson::value val, const char* label, Vector3f &param) {
    if (!val.is<picojson::array>() || !val.contains(2) || val.contains(3)) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    for (uint8_t j=0; j<3; j++) {
        parse_float(val.get(j), label, param[j]);
    }
}

#endif

/*
  initialise the frame
 */
#if AP_SIM_ENABLED
void Frame::init(const char *frame_str, Battery *_battery)
{
    model = default_model;
    battery = _battery;

#if USE_PICOJSON
    const char *colon = strchr(frame_str, ':');
    size_t slen = strlen(frame_str);
    if (colon != nullptr && slen > 5 && strcmp(&frame_str[slen-5], ".json") == 0) {
        load_frame_params(colon+1);
    }
#endif
    mass = model.mass;

    const float drag_force = model.mass * GRAVITY_MSS * tanf(radians(model.refAngle));

    const float cos_tilt = cosf(radians(model.refAngle));
    const float airspeed_bf = model.refSpd * cos_tilt;
    const float ref_thrust = model.mass * GRAVITY_MSS / cos_tilt;
    float ref_air_density = get_air_density(model.refAlt);

    const float momentum_drag = cos_tilt * model.mdrag_coef * airspeed_bf * sqrtf(ref_thrust * ref_air_density * model.disc_area);

    if (momentum_drag > drag_force) {
        model.mdrag_coef *= drag_force / momentum_drag;
        areaCd = 0.0;
        ::printf("Suggested EK3_DRAG_BCOEF_* = 0, EK3_DRAG_MCOEF = %.3f\n", (momentum_drag / (model.mass * airspeed_bf)) * sqrtf(1.225f / ref_air_density));
    } else {
        areaCd = (drag_force - momentum_drag) / (0.5f * ref_air_density * sq(model.refSpd));
        ::printf("Suggested EK3_DRAG_BCOEF_* = %.3f, EK3_DRAG_MCOEF = %.3f\n", model.mass / areaCd, (momentum_drag / (model.mass * airspeed_bf)) * sqrtf(1.225f / ref_air_density));
    }

    terminal_rotation_rate = model.refRotRate;

    float hover_thrust = mass * GRAVITY_MSS;
    float hover_power = model.refCurrent * model.refVoltage;
    float hover_velocity_out = 2 * hover_power / hover_thrust;
    float effective_disc_area = hover_thrust / (0.5 * ref_air_density * sq(hover_velocity_out));
    float velocity_max = hover_velocity_out / sqrtf(model.hoverThrOut);
    float effective_prop_area = effective_disc_area / num_motors;
    float true_prop_area = model.disc_area / num_motors;

    // power_factor is ratio of power consumed per newton of thrust
    float power_factor = hover_power / hover_thrust;

    battery->setup(model.battCapacityAh, model.refBatRes, model.maxVoltage);

    if (uint8_t(model.num_motors) != num_motors) {
        ::printf("Warning model expected %u motors and got %u\n", uint8_t(model.num_motors), num_motors);
    }

    for (uint8_t i=0; i<num_motors; i++) {
        motors[i].setup_params(model.pwmMin, model.pwmMax, model.spin_min, model.spin_max, model.propExpo, model.slew_max,
                               model.diagonal_size, power_factor, model.maxVoltage, effective_prop_area, velocity_max,
                               model.motor_pos[i], model.motor_thrust_vec[i], model.yaw_factor[i], true_prop_area,
                               model.mdrag_coef);
    }

    if (is_zero(model.moment_of_inertia.x) || is_zero(model.moment_of_inertia.y) || is_zero(model.moment_of_inertia.z)) {
        // if no inertia provided, assume 50% of mass on ring around center
        model.moment_of_inertia.x = model.mass * 0.25 * sq(model.diagonal_size*0.5);
        model.moment_of_inertia.y = model.moment_of_inertia.x;
        model.moment_of_inertia.z = model.mass * 0.5 * sq(model.diagonal_size*0.5);
    }

    // setup reasonable defaults for battery
    AP_Param::set_default_by_name("SIM_BATT_VOLTAGE", model.maxVoltage);
    AP_Param::set_default_by_name("SIM_BATT_CAP_AH", model.battCapacityAh);
    if (model.battCapacityAh > 0) {
        AP_Param::set_default_by_name("BATT_CAPACITY", model.battCapacityAh*1000);
    }
}

/*
  find a frame by name
 */
Frame *Frame::find_frame(const char *name)
{
    for (uint8_t i=0; i < ARRAY_SIZE(supported_frames); i++) {
        // do partial name matching to allow for frame variants
        if (strncasecmp(name, supported_frames[i].name, strlen(supported_frames[i].name)) == 0) {
            return &supported_frames[i];
        }
    }
    return nullptr;
}

// calculate rotational and linear accelerations
void Frame::calculate_forces(const Aircraft &aircraft,
                             const struct sitl_input &input,
                             Vector3f &rot_accel,
                             Vector3f &body_accel,
                             float* rpm,
                             bool use_drag)
{
    Vector3f thrust; // newtons
    Vector3f torque;

    const float air_density = get_air_density(aircraft.get_location().alt*0.01);
    const Vector3f gyro = aircraft.get_gyro();

    Vector3f vel_air_bf = aircraft.get_dcm().transposed() * aircraft.get_velocity_air_ef();

    float current = 0;
    for (uint8_t i=0; i<num_motors; i++) {
        Vector3f mtorque, mthrust;
        motors[i].calculate_forces(input, motor_offset, mtorque, mthrust, vel_air_bf, gyro, air_density, battery->get_voltage(), use_drag);
        current += motors[i].get_current();
        torque += mtorque;
        thrust += mthrust;
        // simulate motor rpm
        if (!is_zero(AP::sitl()->vibe_motor)) {
            rpm[motor_offset+i] = motors[i].get_command() * AP::sitl()->vibe_motor * 60.0f;
        }
    }

    // calculate total rotational acceleration
    rot_accel.x = torque.x / model.moment_of_inertia.x;
    rot_accel.y = torque.y / model.moment_of_inertia.y;
    rot_accel.z = torque.z / model.moment_of_inertia.z;

    if (terminal_rotation_rate > 0) {
        // rotational air resistance
        rot_accel.x -= gyro.x * radians(400.0) / terminal_rotation_rate;
        rot_accel.y -= gyro.y * radians(400.0) / terminal_rotation_rate;
        rot_accel.z -= gyro.z * radians(400.0) / terminal_rotation_rate;
    }

    if (use_drag) {
        // use the model params to calculate drag
        Vector3f drag_bf;
        drag_bf.x = areaCd * 0.5f * air_density * sq(vel_air_bf.x);
        if (is_negative(vel_air_bf.x)) {
            drag_bf.x = -drag_bf.x;
        }

        drag_bf.y = areaCd * 0.5f * air_density * sq(vel_air_bf.y);
        if (is_negative(vel_air_bf.y)) {
            drag_bf.y = -drag_bf.y;
        }

        drag_bf.z = areaCd * 0.5f * air_density * sq(vel_air_bf.z);
        if (is_negative(vel_air_bf.z)) {
            drag_bf.z = -drag_bf.z;
        }

        thrust -= drag_bf;
    }

    body_accel = thrust/aircraft.gross_mass();
}


// calculate current and voltage
void Frame::current_and_voltage(float &voltage, float &current)
{
    float param_voltage = AP::sitl()->batt_voltage;
    if (!is_equal(last_param_voltage,param_voltage)) {
        battery->init_voltage(param_voltage);
        last_param_voltage = param_voltage;
    }
    voltage = battery->get_voltage();
    current = 0;
    for (uint8_t i=0; i<num_motors; i++) {
        current += motors[i].get_current();
    }
}
#endif // AP_SIM_ENABLED
