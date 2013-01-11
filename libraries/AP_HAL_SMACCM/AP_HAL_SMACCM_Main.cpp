/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * AP_HAL_SMACCM_Main.cpp --- AP_HAL_SMACCM main task implementation.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 20 December 2012
 */

#include <AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM

#include <AP_HAL_SMACCM.h>
#include <AP_HAL_SMACCM_Main.h>
#include <FreeRTOS.h>
#include <task.h>

static xTaskHandle g_main_task;

// These must be defined in the main ".pde" file.
extern const AP_HAL::HAL& hal;
extern void setup();
extern void loop();

static void main_task(void *arg)
{
  hal.init(0, NULL);
  setup();
  hal.scheduler->system_initialized();

  for (;;)
    loop();
}

void SMACCM::hal_main()
{
  xTaskCreate(main_task, (signed char *)"main", 1024, NULL, 0, &g_main_task);
  vTaskStartScheduler();

  for (;;)
    ;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
