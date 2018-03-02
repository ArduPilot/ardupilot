/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 * Author: 4712
*/
#pragma once

uint8_t Stk_SignOn(void);
uint8_t Stk_ConnectEx(uint8_32_u *pDeviceInfo);
uint8_t Stk_ReadEEprom(ioMem_t *pMem);
uint8_t Stk_WriteEEprom(ioMem_t *pMem);
uint8_t Stk_ReadFlash(ioMem_t *pMem);
uint8_t Stk_WriteFlash(ioMem_t *pMem);
uint8_t Stk_Chip_Erase(void);

