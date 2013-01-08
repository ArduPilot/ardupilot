
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
