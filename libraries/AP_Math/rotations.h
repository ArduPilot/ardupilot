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
enum Rotation {
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
    ROTATION_PITCH_180_YAW_90    = 26,
    ROTATION_PITCH_180_YAW_270   = 27,
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
    ROTATION_ROLL_90_PITCH_68_YAW_293 = 38,
    ROTATION_MAX
};
/*
Here are the same values in a form sutable for a @Values attribute in
auto documentation:

@Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180
 */
