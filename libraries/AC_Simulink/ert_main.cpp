//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ert_main.cpp
//
// Code generated for Simulink model 'arduAttCont'.
//
// Model version                  : 1.3
// Simulink Coder version         : 9.6 (R2021b) 14-May-2021
// C/C++ source code generated on : Sun Sep 11 04:36:15 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives: Unspecified
// Validation result: Not run
//

///////
// This line is added intentionally after code generation to prevent compilation of this file for hardware target
#if 0
//////

#include <stdio.h>              // This example main program uses printf/fflush
#include "arduAttCont.h"               // Model's header file

static arduAttContModelClass arduAttCont_Obj;// Instance of model class

// '<Root>/attiude_error'
static real32_T arg_attiude_error[3]{ 0.0F, 0.0F, 0.0F };

// '<Root>/rate_ff'
static real32_T arg_rate_ff[3]{ 0.0F, 0.0F, 0.0F };

// '<Root>/rate_meas'
static real32_T arg_rate_meas[3]{ 0.0F, 0.0F, 0.0F };

// '<Root>/Out1'
static real32_T arg_Out1[3];

//
// Associating rt_OneStep with a real-time clock or interrupt service routine
// is what makes the generated code "real-time".  The function rt_OneStep is
// always associated with the base rate of the model.  Subrates are managed
// by the base rate from inside the generated code.  Enabling/disabling
// interrupts and floating point context switches are target specific.  This
// example code indicates where these should take place relative to executing
// the generated code step function.  Overrun behavior should be tailored to
// your application needs.  This example simply sets an error status in the
// real-time model and returns from rt_OneStep.
//
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag{ false };

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(arduAttCont_Obj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  arduAttCont_Obj.step(arg_attiude_error, arg_rate_ff, arg_rate_meas, arg_Out1);

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

//
// The example "main" function illustrates what is required by your
// application code to initialize, execute, and terminate the generated code.
// Attaching rt_OneStep to a real-time clock is target specific.  This example
// illustrates how you do this relative to initializing the model.
//
int_T main(int_T argc, const char *argv[])
{
  // Unused arguments
  (void)(argc);
  (void)(argv);

  // Initialize model
  arduAttCont_Obj.initialize();

  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.0025 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep();

  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((nullptr));
  while (rtmGetErrorStatus(arduAttCont_Obj.getRTM()) == (nullptr)) {
    //  Perform application tasks here
  }

  // Disable rt_OneStep here
  // Terminate model
  arduAttCont_Obj.terminate();
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//

///////
// This line is added intentionally after code generation to prevent compilation of this file for hardware target
#endif
//////