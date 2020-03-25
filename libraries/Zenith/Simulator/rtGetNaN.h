
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include "defines.h"

/*
 * rtGetNaN.h
 *
 * Code generation for model "Simulator".
 *
 * Model version              : 0.554
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C++ source code generated on : Tue Mar 24 14:39:15 2020
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_rtGetNaN_h_
#define RTW_HEADER_rtGetNaN_h_
#include <stddef.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#ifdef __cplusplus

extern "C" {

#endif

  extern real_T rtGetNaN(void);
  extern real32_T rtGetNaNF(void);

#ifdef __cplusplus

}                                      /* extern "C" */
#endif
#endif                                 /* RTW_HEADER_rtGetNaN_h_ */

#pragma GCC diagnostic pop
