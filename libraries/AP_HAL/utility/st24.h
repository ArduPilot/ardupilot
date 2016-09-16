// -*- tab-width: 8; Mode: C++; c-basic-offset: 8; indent-tabs-mode: -*- t -*-
/*
  st24 decoder, based on PX4Firmware/src/rc/lib/rc/st24.c from PX4Firmware
  modified for use in AP_HAL_* by Andrew Tridgell
 */
/****************************************************************************
 *
 *	Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file st24.h
 *
 * RC protocol definition for Yuneec ST24 transmitter
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#pragma once

/**
 * Decoder for ST24 protocol
 *
 * @param byte current char to read
 * @param rssi pointer to a byte where the RSSI value is written back to
 * @param rx_count pointer to a byte where the receive count of packets signce last wireless frame is written back to
 * @param channels pointer to a datastructure of size max_chan_count where channel values (12 bit) are written back to
 * @param max_chan_count maximum channels to decode - if more channels are decoded, the last n are skipped and success (0) is returned
 * @return 0 for success (a decoded packet), 1 for no packet yet (accumulating), 2 for unknown packet, 3 for out of sync, 4 for checksum error
 */
int st24_decode(uint8_t byte, uint8_t *rssi, uint8_t *rx_count, uint16_t *channel_count,
                uint16_t *channels, uint16_t max_chan_count);
