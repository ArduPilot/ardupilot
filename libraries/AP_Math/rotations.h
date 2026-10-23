/*
 * rotations.h
 * Copyright (C) Andrew Tridgell 2012
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

// these rotations form a full set - every rotation in the following
// list when combined with another in the list forms an entry which is
// also in the list. This is an important property. Please run the
// rotations test suite if you add to the list.

// NOTE!! these rotation values are stored to EEPROM, so be careful not to
// change the numbering of any existing entry when adding a new entry.
enum Rotation : uint8_t {
    ROTATION_NONE                = 0,
    ROTATION_YAW_45              = 1,
    ROTATION_YAW_90              = 2,
    ROTATION_YAW_135             = 3,
    ROTATION_YAW_180             = 4,
    ROTATION_YAW_225             = 5,
    ROTATION_YAW_270             = 6,
    ROTATION_YAW_315             = 7,
    ROTATION_ROLL_180            = 8,
    ROTATION_ROLL_180_YAW_45     = 9,
    ROTATION_ROLL_180_YAW_90     = 10,
    ROTATION_ROLL_180_YAW_135    = 11,
    ROTATION_PITCH_180           = 12,
    ROTATION_ROLL_180_YAW_225    = 13,
    ROTATION_ROLL_180_YAW_270    = 14,
    ROTATION_ROLL_180_YAW_315    = 15,
    ROTATION_ROLL_90             = 16,
    ROTATION_ROLL_90_YAW_45      = 17,
    ROTATION_ROLL_90_YAW_90      = 18,
    ROTATION_ROLL_90_YAW_135     = 19,
    ROTATION_ROLL_270            = 20,
    ROTATION_ROLL_270_YAW_45     = 21,
    ROTATION_ROLL_270_YAW_90     = 22,
    ROTATION_ROLL_270_YAW_135    = 23,
    ROTATION_PITCH_90            = 24,
    ROTATION_PITCH_270           = 25,
    ROTATION_PITCH_180_YAW_90    = 26, // same as ROTATION_ROLL_180_YAW_270
    ROTATION_PITCH_180_YAW_270   = 27, // same as ROTATION_ROLL_180_YAW_90
    ROTATION_ROLL_90_PITCH_90    = 28,
    ROTATION_ROLL_180_PITCH_90   = 29,
    ROTATION_ROLL_270_PITCH_90   = 30,
    ROTATION_ROLL_90_PITCH_180   = 31,
    ROTATION_ROLL_270_PITCH_180  = 32,
    ROTATION_ROLL_90_PITCH_270   = 33,
    ROTATION_ROLL_180_PITCH_270  = 34,
    ROTATION_ROLL_270_PITCH_270  = 35,
    ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
    ROTATION_ROLL_90_YAW_270     = 37,
    ROTATION_ROLL_90_PITCH_68_YAW_293 = 38, // this is actually, roll 90, pitch 68.8, yaw 293.3
    ROTATION_PITCH_315           = 39,
    ROTATION_ROLL_90_PITCH_315   = 40,
    ROTATION_PITCH_7             = 41,
    ROTATION_ROLL_45             = 42,
    ROTATION_ROLL_315            = 43,
    ///////////////////////////////////////////////////////////////////////
    // Do not add more rotations without checking that there is not a conflict
    // with the MAVLink spec. MAV_SENSOR_ORIENTATION is expected to match our
    // list of rotations here. If a new rotation is added it needs to be added
    // to the MAVLink messages as well.
    ///////////////////////////////////////////////////////////////////////
    ROTATION_MAX,
    ROTATION_CUSTOM_OLD          = 100,
    ROTATION_CUSTOM_1            = 101,
    ROTATION_CUSTOM_2            = 102,
    ROTATION_CUSTOM_END,
};


// definitions used by quaterion and vector3f
#define HALF_SQRT_2 0.70710678118654752440084436210485

/*
Here are the same values in a form suitable for a @Values attribute in
auto documentation:

@Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Yaw45Roll180,10:Yaw90Roll180,11:Yaw135Roll180,12:Pitch180,13:Yaw225Roll180,14:Yaw270Roll180,15:Yaw315Roll180,16:Roll90,17:Yaw45Roll90,18:Yaw90Roll90,19:Yaw135Roll90,20:Roll270,21:Yaw45Roll270,22:Yaw90Roll270,23:Yaw135Roll270,24:Pitch90,25:Pitch270,26:Yaw90Pitch180,27:Yaw270Pitch180,28:Pitch90Roll90,29:Pitch90Roll180,30:Pitch90Roll270,31:Pitch180Roll90,32:Pitch180Roll270,33:Pitch270Roll90,34:Pitch270Roll180,35:Pitch270Roll270,36:Yaw90Pitch180Roll90,37:Yaw270Roll90,38:Yaw293Pitch68Roll180,39:Pitch315,40:Pitch315Roll90,42:Roll45,43:Roll315,100:Custom 4.1 and older,101:Custom 1,102:Custom 2
 */
