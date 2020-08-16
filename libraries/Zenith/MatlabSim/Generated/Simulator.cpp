
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include "defines.h"

/*
 * Simulator.cpp
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

#include "Simulator.h"
#include "Simulator_private.h"

const CmdBus Simulator_rtZCmdBus = {
  0.0,                                 /* thr */
  0.0,                                 /* ail */
  0.0,                                 /* elev */
  0.0,                                 /* rud */
  0.0,                                 /* flaps */
  0.0,                                 /* ailOut */
  0.0                                  /* thrDiff */
} ;                                    /* CmdBus ground */

const ACBus Simulator_rtZACBus = {
  0.0,                                 /* time */

  {
    0.0, 0.0, 0.0 }
  ,                                    /* Wb */

  {
    0.0, 0.0, 0.0 }
  ,                                    /* Ve */

  {
    0.0, 0.0, 0.0 }
  ,                                    /* Ab */

  {
    0.0, 0.0, 0.0, 0.0 }
  ,                                    /* quat */

  {
    0.0, 0.0, 0.0 }
  /* Xe */
} ;                                    /* ACBus ground */

static void rate_scheduler(RT_MODEL_Simulator_T *const Simulator_M);

/*     Initialize pressure and temperature tables. */
void InitCalcAtmosCOESA(real_T *temperature76, real_T *pressureRatio76)
{
  if (temperature76[0] != TEMPERATURE0 ) {
    int_T k;
    temperature76[0] = TEMPERATURE0;
    pressureRatio76[0] = 1.0;

    /* set up the data at the 1976 altitude breakpoints */
    for (k=0; k<(NUM1976PTS-1); k++) {
      if (tempGradient76[k] != 0.0) {
        temperature76[k+1] = temperature76[k] +
          tempGradient76[k]*(altitude76[k+1] - altitude76[k]);
        pressureRatio76[k+1] = pressureRatio76[k] *
          std::exp(std::log(temperature76[k]/temperature76[k+1]) * GMR/
                   tempGradient76[k]);
      } else {
        temperature76[k+1] = temperature76[k];
        pressureRatio76[k+1] = pressureRatio76[k] *
          std::exp((-GMR)*(altitude76[k+1] - altitude76[k])/temperature76[k]);
      }
    }
  }
}

/*
 *     Using cached pressure and temperature tables, find the
 *     working interval and perform logarithmic interpolation.
 */
void CalcAtmosCOESA(const real_T *altitude, real_T *temp, real_T *pressure,
                    real_T *density, real_T *speedofsound, real_T *temperature76,
                    real_T *pressureRatio76, int_T numPoints)
{
  int_T i;
  for (i=0; i < numPoints; i++) {
    int_T bottom = 0;
    int_T top = NUM1976PTS-1;
    int_T idx;

    /* Find altitude interval using binary search
     *
     * Deal with the extreme cases first:
     *   if altitude <= altitude76[bottom] then return idx = bottom
     *   if altitude >= altitude76[top]    then return idx = top
     */
    if (altitude[i] <= altitude76[bottom]) {
      idx = bottom;
    } else if (altitude[i] >= altitude76[top]) {
      idx = NUM1976PTS-2;
    } else {
      for (;;) {
        idx = (bottom + top)/2;
        if (altitude[i] < altitude76[idx]) {
          top = idx - 1;
        } else if (altitude[i] >= altitude76[idx+1]) {
          bottom = idx + 1;
        } else {
          /* we have altitude76[idx] <= altitude[i] < altitude76[idx+1],
           * so break and just use idx
           */
          break;
        }
      }
    }

    /* Interval has been obtained, now do linear temperature
     * interpolation and log pressure interpolation.
     */
    if (tempGradient76[idx] != 0.0 ) {
      temp[i] = temperature76[idx] +
        tempGradient76[idx] * (altitude[i] - altitude76[idx]);
      pressure[i] = PRESSURE0 * pressureRatio76[idx] *
        (rt_powd_snf(temperature76[idx]/temp[i], GMR/tempGradient76[idx]));
    } else {
      temp[i] = temperature76[idx];
      pressure[i] = PRESSURE0 * pressureRatio76[idx] *
        std::exp((-GMR)*(altitude[i] - altitude76[idx]) / temperature76[idx]);
    }

    density[i] = pressure[i] / ((R_HAT/MOL_WT)*temp[i]);
    speedofsound[i] = std::sqrt(GAMMA*temp[i]*(R_HAT/MOL_WT));
  }
}

real_T look1_binlc(real_T u0, const real_T bp0[], const real_T table[], uint32_T
                   maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = 1.0;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

uint32_T plook_bincpa(real_T u, const real_T bp[], uint32_T maxIndex, real_T
                      *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d_prevIdx(u, bp, *prevIndex, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex;
    *fraction = 0.0;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

real_T intrp2d_la(const uint32_T bpIndex[], const real_T frac[], const real_T
                  table[], const uint32_T stride, const uint32_T maxIndex[])
{
  real_T y;
  real_T yR_1d;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'wrapping'
   */
  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  if (bpIndex[0U] == maxIndex[0U]) {
    y = table[offset_1d];
  } else {
    y = (table[offset_1d + 1U] - table[offset_1d]) * frac[0U] + table[offset_1d];
  }

  if (bpIndex[1U] == maxIndex[1U]) {
  } else {
    offset_1d += stride;
    if (bpIndex[0U] == maxIndex[0U]) {
      yR_1d = table[offset_1d];
    } else {
      yR_1d = (table[offset_1d + 1U] - table[offset_1d]) * frac[0U] +
        table[offset_1d];
    }

    y += (yR_1d - y) * frac[1U];
  }

  return y;
}

uint32_T plook_binc(real_T u, const real_T bp[], uint32_T maxIndex, real_T
                    *fraction)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d(u, bp, maxIndex >> 1U, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = 1.0;
  }

  return bpIndex;
}

real_T intrp1d_l(uint32_T bpIndex, real_T frac, const real_T table[])
{
  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  return (table[bpIndex + 1U] - table[bpIndex]) * frac + table[bpIndex];
}

uint32_T plook_binx(real_T u, const real_T bp[], uint32_T maxIndex, real_T
                    *fraction)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = (u - bp[0U]) / (bp[1U] - bp[0U]);
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d(u, bp, maxIndex >> 1U, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = (u - bp[maxIndex - 1U]) / (bp[maxIndex] - bp[maxIndex - 1U]);
  }

  return bpIndex;
}

real_T intrp3d_l(const uint32_T bpIndex[], const real_T frac[], const real_T
                 table[], const uint32_T stride[])
{
  real_T yL_2d;
  uint32_T offset_2d;
  real_T yL_1d;
  uint32_T offset_0d;

  /* Column-major Interpolation 3-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  offset_2d = (bpIndex[2U] * stride[2U] + bpIndex[1U] * stride[1U]) + bpIndex[0U];
  yL_1d = (table[offset_2d + 1U] - table[offset_2d]) * frac[0U] +
    table[offset_2d];
  offset_0d = offset_2d + stride[1U];
  yL_2d = (((table[offset_0d + 1U] - table[offset_0d]) * frac[0U] +
            table[offset_0d]) - yL_1d) * frac[1U] + yL_1d;
  offset_2d += stride[2U];
  yL_1d = (table[offset_2d + 1U] - table[offset_2d]) * frac[0U] +
    table[offset_2d];
  offset_0d = offset_2d + stride[1U];
  return (((((table[offset_0d + 1U] - table[offset_0d]) * frac[0U] +
             table[offset_0d]) - yL_1d) * frac[1U] + yL_1d) - yL_2d) * frac[2U]
    + yL_2d;
}

uint32_T binsearch_u32d_prevIdx(real_T u, const real_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T found;

  /* Binary Search using Previous Index */
  bpIndex = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  found = 0U;
  while (found == 0U) {
    if (u < bp[bpIndex]) {
      iRght = bpIndex - 1U;
      bpIndex = (iRght + iLeft) >> 1U;
    } else if (u < bp[bpIndex + 1U]) {
      found = 1U;
    } else {
      iLeft = bpIndex + 1U;
      bpIndex = (iRght + iLeft) >> 1U;
    }
  }

  return bpIndex;
}

uint32_T binsearch_u32d(real_T u, const real_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T iRght;
  uint32_T bpIdx;

  /* Binary Search */
  bpIdx = startIndex;
  bpIndex = 0U;
  iRght = maxIndex;
  while (iRght - bpIndex > 1U) {
    if (u < bp[bpIdx]) {
      iRght = bpIdx;
    } else {
      bpIndex = bpIdx;
    }

    bpIdx = (iRght + bpIndex) >> 1U;
  }

  return bpIndex;
}

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(RT_MODEL_Simulator_T *const Simulator_M)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (Simulator_M->Timing.TaskCounters.TID[2])++;
  if ((Simulator_M->Timing.TaskCounters.TID[2]) > 99) {/* Sample time: [0.1s, 0.0s] */
    Simulator_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/* State reduction function */
void local_stateReduction(real_T* x, int_T* p, int_T n, real_T* r)
{
  int_T i, j;
  for (i = 0, j = 0; i < n; ++i, ++j) {
    int_T k = p[i];
    real_T lb = r[j++];
    real_T xk = x[k]-lb;
    real_T rk = r[j]-lb;
    int_T q = (int_T) std::floor(xk/rk);
    if (q) {
      x[k] = xk-q*rk+lb;
    }
  }
}

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
void SimulatorModelClass::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = static_cast<ODE1_IntgData *>(rtsiGetSolverData(si));
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 36;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  Simulator_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  local_stateReduction(x, rtsiGetPeriodicContStateIndices(si), 3,
                       rtsiGetPeriodicContStateRanges(si));
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * Output and update for trigger system:
 *    '<S24>/Triggered Subsystem'
 *    '<S41>/Triggered Subsystem'
 *    '<S49>/Triggered Subsystem'
 *    '<S64>/Triggered Subsystem'
 */
void SimulatorModelClass::Simulator_TriggeredSubsystem(const boolean_T
  rtu_Trigger[3], ZCE_TriggeredSubsystem_Simula_T *localZCE)
{
  /* Outputs for Triggered SubSystem: '<S24>/Triggered Subsystem' incorporates:
   *  TriggerPort: '<S27>/Trigger'
   */
  if (rtmIsMajorTimeStep((&Simulator_M))) {
    localZCE->TriggeredSubsystem_Trig_ZCE[0] = rtu_Trigger[0];
    localZCE->TriggeredSubsystem_Trig_ZCE[1] = rtu_Trigger[1];
    localZCE->TriggeredSubsystem_Trig_ZCE[2] = rtu_Trigger[2];
  }

  /* End of Outputs for SubSystem: '<S24>/Triggered Subsystem' */
}

/*
 * System initialize for enable system:
 *    '<S23>/NEGATIVE Edge'
 *    '<S40>/NEGATIVE Edge'
 *    '<S48>/NEGATIVE Edge'
 *    '<S63>/NEGATIVE Edge'
 */
void SimulatorModelClass::Simulator_NEGATIVEEdge_Init(B_NEGATIVEEdge_Simulator_T
  *localB)
{
  /* SystemInitialize for Outport: '<S25>/OUT' */
  localB->RelationalOperator1[0] = false;
  localB->RelationalOperator1[1] = false;
  localB->RelationalOperator1[2] = false;
}

/*
 * Disable for enable system:
 *    '<S23>/NEGATIVE Edge'
 *    '<S40>/NEGATIVE Edge'
 *    '<S48>/NEGATIVE Edge'
 *    '<S63>/NEGATIVE Edge'
 */
void SimulatorModelClass::Simulator_NEGATIVEEdge_Disable
  (DW_NEGATIVEEdge_Simulator_T *localDW)
{
  localDW->NEGATIVEEdge_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S23>/NEGATIVE Edge'
 *    '<S40>/NEGATIVE Edge'
 *    '<S48>/NEGATIVE Edge'
 *    '<S63>/NEGATIVE Edge'
 */
void SimulatorModelClass::Simulator_NEGATIVEEdge(real_T rtu_Enable, const
  boolean_T rtu_IN[3], const boolean_T rtu_INprevious[3],
  B_NEGATIVEEdge_Simulator_T *localB, DW_NEGATIVEEdge_Simulator_T *localDW)
{
  /* Outputs for Enabled SubSystem: '<S23>/NEGATIVE Edge' incorporates:
   *  EnablePort: '<S25>/Enable'
   */
  if ((rtmIsMajorTimeStep((&Simulator_M)) &&
       (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) && rtmIsMajorTimeStep
      ((&Simulator_M))) {
    if (rtu_Enable > 0.0) {
      localDW->NEGATIVEEdge_MODE = true;
    } else {
      if (localDW->NEGATIVEEdge_MODE) {
        Simulator_NEGATIVEEdge_Disable(localDW);
      }
    }
  }

  if (localDW->NEGATIVEEdge_MODE) {
    /* RelationalOperator: '<S25>/Relational Operator1' */
    localB->RelationalOperator1[0] = (static_cast<int32_T>(rtu_INprevious[0]) >
      static_cast<int32_T>(rtu_IN[0]));
    localB->RelationalOperator1[1] = (static_cast<int32_T>(rtu_INprevious[1]) >
      static_cast<int32_T>(rtu_IN[1]));
    localB->RelationalOperator1[2] = (static_cast<int32_T>(rtu_INprevious[2]) >
      static_cast<int32_T>(rtu_IN[2]));
  }

  /* End of Outputs for SubSystem: '<S23>/NEGATIVE Edge' */
}

/*
 * System initialize for enable system:
 *    '<S23>/POSITIVE Edge'
 *    '<S40>/POSITIVE Edge'
 *    '<S48>/POSITIVE Edge'
 *    '<S63>/POSITIVE Edge'
 */
void SimulatorModelClass::Simulator_POSITIVEEdge_Init(B_POSITIVEEdge_Simulator_T
  *localB)
{
  /* SystemInitialize for Outport: '<S26>/OUT' */
  localB->RelationalOperator1[0] = false;
  localB->RelationalOperator1[1] = false;
  localB->RelationalOperator1[2] = false;
}

/*
 * Disable for enable system:
 *    '<S23>/POSITIVE Edge'
 *    '<S40>/POSITIVE Edge'
 *    '<S48>/POSITIVE Edge'
 *    '<S63>/POSITIVE Edge'
 */
void SimulatorModelClass::Simulator_POSITIVEEdge_Disable
  (DW_POSITIVEEdge_Simulator_T *localDW)
{
  localDW->POSITIVEEdge_MODE = false;
}

/*
 * Output and update for enable system:
 *    '<S23>/POSITIVE Edge'
 *    '<S40>/POSITIVE Edge'
 *    '<S48>/POSITIVE Edge'
 *    '<S63>/POSITIVE Edge'
 */
void SimulatorModelClass::Simulator_POSITIVEEdge(real_T rtu_Enable, const
  boolean_T rtu_IN[3], const boolean_T rtu_INprevious[3],
  B_POSITIVEEdge_Simulator_T *localB, DW_POSITIVEEdge_Simulator_T *localDW)
{
  /* Outputs for Enabled SubSystem: '<S23>/POSITIVE Edge' incorporates:
   *  EnablePort: '<S26>/Enable'
   */
  if ((rtmIsMajorTimeStep((&Simulator_M)) &&
       (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) && rtmIsMajorTimeStep
      ((&Simulator_M))) {
    if (rtu_Enable > 0.0) {
      localDW->POSITIVEEdge_MODE = true;
    } else {
      if (localDW->POSITIVEEdge_MODE) {
        Simulator_POSITIVEEdge_Disable(localDW);
      }
    }
  }

  if (localDW->POSITIVEEdge_MODE) {
    /* RelationalOperator: '<S26>/Relational Operator1' */
    localB->RelationalOperator1[0] = (static_cast<int32_T>(rtu_INprevious[0]) <
      static_cast<int32_T>(rtu_IN[0]));
    localB->RelationalOperator1[1] = (static_cast<int32_T>(rtu_INprevious[1]) <
      static_cast<int32_T>(rtu_IN[1]));
    localB->RelationalOperator1[2] = (static_cast<int32_T>(rtu_INprevious[2]) <
      static_cast<int32_T>(rtu_IN[2]));
  }

  /* End of Outputs for SubSystem: '<S23>/POSITIVE Edge' */
}

/*
 * System initialize for enable system:
 *    '<S253>/Distance into gust (y)'
 *    '<S253>/Distance into gust (z)'
 */
void SimulatorModelClass::Simulato_Distanceintogusty_Init
  (B_Distanceintogusty_Simulator_T *localB, X_Distanceintogusty_Simulator_T
   *localX)
{
  /* InitializeConditions for Integrator: '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
  localX->DistanceintoGustxLimitedtogustl = 0.0;

  /* SystemInitialize for Outport: '<S257>/x' */
  localB->DistanceintoGustxLimitedtogustl = 0.0;
}

/*
 * System reset for enable system:
 *    '<S253>/Distance into gust (y)'
 *    '<S253>/Distance into gust (z)'
 */
void SimulatorModelClass::Simulat_Distanceintogusty_Reset
  (X_Distanceintogusty_Simulator_T *localX)
{
  /* InitializeConditions for Integrator: '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
  localX->DistanceintoGustxLimitedtogustl = 0.0;
}

/*
 * Disable for enable system:
 *    '<S253>/Distance into gust (y)'
 *    '<S253>/Distance into gust (z)'
 */
void SimulatorModelClass::Simul_Distanceintogusty_Disable
  (DW_Distanceintogusty_Simulato_T *localDW)
{
  localDW->Distanceintogusty_MODE = false;
}

/*
 * Outputs for enable system:
 *    '<S253>/Distance into gust (y)'
 *    '<S253>/Distance into gust (z)'
 */
void SimulatorModelClass::Simulator_Distanceintogusty(boolean_T rtu_Enable,
  B_Distanceintogusty_Simulator_T *localB, DW_Distanceintogusty_Simulato_T
  *localDW, X_Distanceintogusty_Simulator_T *localX, real_T rtp_d_m)
{
  /* Outputs for Enabled SubSystem: '<S253>/Distance into gust (y)' incorporates:
   *  EnablePort: '<S257>/Enable'
   */
  if ((rtmIsMajorTimeStep((&Simulator_M)) &&
       (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) && rtmIsMajorTimeStep
      ((&Simulator_M))) {
    if (rtu_Enable) {
      if (!localDW->Distanceintogusty_MODE) {
        Simulat_Distanceintogusty_Reset(localX);
        localDW->Distanceintogusty_MODE = true;
      }
    } else {
      if (localDW->Distanceintogusty_MODE) {
        Simul_Distanceintogusty_Disable(localDW);
      }
    }
  }

  if (localDW->Distanceintogusty_MODE) {
    /* Integrator: '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
    /* Limited  Integrator  */
    if (localX->DistanceintoGustxLimitedtogustl >= rtp_d_m) {
      localX->DistanceintoGustxLimitedtogustl = rtp_d_m;
    } else {
      if (localX->DistanceintoGustxLimitedtogustl <= 0.0) {
        localX->DistanceintoGustxLimitedtogustl = 0.0;
      }
    }

    localB->DistanceintoGustxLimitedtogustl =
      localX->DistanceintoGustxLimitedtogustl;

    /* End of Integrator: '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
  }

  /* End of Outputs for SubSystem: '<S253>/Distance into gust (y)' */
}

/*
 * Derivatives for enable system:
 *    '<S253>/Distance into gust (y)'
 *    '<S253>/Distance into gust (z)'
 */
void SimulatorModelClass::Simulat_Distanceintogusty_Deriv(real_T rtu_V,
  DW_Distanceintogusty_Simulato_T *localDW, X_Distanceintogusty_Simulator_T
  *localX, XDot_Distanceintogusty_Simula_T *localXdot, real_T rtp_d_m)
{
  boolean_T lsat;
  boolean_T usat;
  if (localDW->Distanceintogusty_MODE) {
    /* Derivatives for Integrator: '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
    lsat = (localX->DistanceintoGustxLimitedtogustl <= 0.0);
    usat = (localX->DistanceintoGustxLimitedtogustl >= rtp_d_m);
    if (((!lsat) && (!usat)) || (lsat && (rtu_V > 0.0)) || (usat && (rtu_V < 0.0)))
    {
      localXdot->DistanceintoGustxLimitedtogustl = rtu_V;
    } else {
      /* in saturation */
      localXdot->DistanceintoGustxLimitedtogustl = 0.0;
    }

    /* End of Derivatives for Integrator: '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
  } else {
    localXdot->DistanceintoGustxLimitedtogustl = 0.0;
  }
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T lo;
  uint32_T hi;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return static_cast<real_T>(*u) * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T y;
  real_T sr;
  real_T si;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = std::sqrt(-2.0 * std::log(si) / si) * sr;
  return y;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

void rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(const real_T u0[3], const real_T u1[9],
  real_T y[3])
{
  real_T A[9];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  real_T maxval;
  real_T a21;
  int32_T rtemp;
  std::memcpy(&A[0], &u1[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(u1[0]);
  a21 = std::abs(u1[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(u1[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = u1[r2] / u1[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
    rtemp = r2 + 1;
    r2 = r3;
    r3 = rtemp - 1;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  y[r1] = u0[0] / A[r1];
  y[r2] = u0[1] - A[r1 + 3] * y[r1];
  y[r3] = u0[2] - A[r1 + 6] * y[r1];
  y[r2] /= A[r2 + 3];
  y[r3] -= A[r2 + 6] * y[r2];
  y[r3] /= A[r3 + 6];
  y[r2] -= A[r3 + 3] * y[r3];
  y[r1] -= y[r3] * A[r3];
  y[r1] -= y[r2] * A[r2];
}

/* Model step function */
void SimulatorModelClass::step()
{
  /* local block i/o variables */
  real_T rtb_Integrator[5];
  real_T rtb_Sum_b[5];
  real_T rtb_Integrator_f[2];
  real_T rtb_Sum_f[2];
  boolean_T rtb_LogicalOperator;
  if (rtmIsMajorTimeStep((&Simulator_M))) {
    /* set solver stop time */
    if (!((&Simulator_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&Simulator_M)->solverInfo, (((&Simulator_M)
        ->Timing.clockTickH0 + 1) * (&Simulator_M)->Timing.stepSize0 *
        4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&Simulator_M)->solverInfo, (((&Simulator_M)
        ->Timing.clockTick0 + 1) * (&Simulator_M)->Timing.stepSize0 +
        (&Simulator_M)->Timing.clockTickH0 * (&Simulator_M)->Timing.stepSize0 *
        4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&Simulator_M))) {
    (&Simulator_M)->Timing.t[0] = rtsiGetT(&(&Simulator_M)->solverInfo);
  }

  {
    boolean_T didZcEventOccur;
    boolean_T rEQ0;
    real_T rtb_Product_ly[3];
    real_T rtb_InterpolationUsingPrelookup;
    real_T rtb_Saturation_m;
    real_T rtb_sigma_ugsigma_vg;
    uint32_T rtb_k_b;
    real_T rtb_VectorConcatenate[9];
    uint32_T rtb_k;
    real_T rtb_Sideslip;
    uint32_T rtb_k_d;
    real_T rtb_f_n;
    real_T rtb_Add_bm;
    real_T rtb_Add_kh;
    real_T rtb_CLtoCz;
    real_T rtb_Add_bc;
    real_T rtb_Add_hq;
    real_T rtb_jxi;
    real_T rtb_ixk;
    real_T rtb_kxj;
    real_T rtb_thetadot;
    real_T rtb_Fcn1_f;
    real_T rtb_Fcn_m;
    real_T rtb_Product_cl[6];
    real_T rtb_Mstabtobody[3];
    real_T rtb_Integrator_i[3];
    real_T rtb_Sum2_m[3];
    boolean_T rtb_LogicalOperator2_p;
    real_T rtb_thr;
    real_T rtb_ail;
    real_T rtb_elev;
    real_T rtb_rud;
    real_T rtb_flaps;
    real_T rtb_ailOut;
    real_T rtb_thrDiff;
    boolean_T rtb_platformStart;
    real_T rtb_Fcn1;
    boolean_T rtb_reset[3];
    real_T rtb_sincos_o1[3];
    boolean_T rtb_LowerRelop1_g[3];
    boolean_T rtb_LogicalOperator3_d[3];
    real_T rtb_Integrator_o[3];
    real_T rtb_Gain3[3];
    real_T rtb_UnitConversion;
    real_T rtb_LowAltitudeScaleLength;
    int8_T rtAction;
    real_T rtb_Saturation[5];
    real_T rtb_time;
    boolean_T rtb_LogicalOperator3_c;
    real_T rtb_UnitConversion_i;
    real_T frac[2];
    uint32_T bpIndex[2];
    real_T rtb_MediumHighAltitudeIntensity;
    real_T frac_0[3];
    uint32_T bpIndex_0[3];
    real_T frac_1[3];
    uint32_T bpIndex_1[3];
    real_T frac_2[3];
    uint32_T bpIndex_2[3];
    real_T frac_3[3];
    uint32_T bpIndex_3[3];
    real_T frac_4[3];
    uint32_T bpIndex_4[3];
    real_T frac_5[3];
    uint32_T bpIndex_5[3];
    real_T frac_6[3];
    uint32_T bpIndex_6[3];
    real_T frac_7[3];
    uint32_T bpIndex_7[3];
    real_T frac_8[3];
    uint32_T bpIndex_8[3];
    real_T frac_9[3];
    uint32_T bpIndex_9[3];
    real_T frac_a[3];
    uint32_T bpIndex_a[3];
    real_T frac_b[3];
    uint32_T bpIndex_b[3];
    real_T frac_c[3];
    uint32_T bpIndex_c[3];
    real_T frac_d[3];
    uint32_T bpIndex_d[3];
    real_T frac_e[3];
    uint32_T bpIndex_e[3];
    real_T frac_f[3];
    uint32_T bpIndex_f[3];
    real_T frac_g[3];
    uint32_T bpIndex_g[3];
    real_T frac_h[3];
    uint32_T bpIndex_h[3];
    real_T frac_i[3];
    uint32_T bpIndex_i[3];
    real_T frac_j[3];
    uint32_T bpIndex_j[3];
    real_T frac_k[3];
    uint32_T bpIndex_k[3];
    real_T frac_l[3];
    uint32_T bpIndex_l[3];
    real_T frac_m[3];
    uint32_T bpIndex_m[3];
    real_T frac_n[3];
    uint32_T bpIndex_n[3];
    real_T frac_o[3];
    uint32_T bpIndex_o[3];
    real_T frac_p[3];
    uint32_T bpIndex_p[3];
    real_T frac_q[3];
    uint32_T bpIndex_q[3];
    real_T frac_r[3];
    uint32_T bpIndex_r[3];
    real_T frac_s[3];
    uint32_T bpIndex_s[3];
    real_T frac_t[3];
    uint32_T bpIndex_t[3];
    real_T frac_u[3];
    uint32_T bpIndex_u[3];
    real_T frac_v[3];
    uint32_T bpIndex_v[3];
    real_T frac_w[3];
    uint32_T bpIndex_w[3];
    real_T frac_x[3];
    uint32_T bpIndex_x[3];
    real_T frac_y[3];
    uint32_T bpIndex_y[3];
    real_T frac_z[3];
    uint32_T bpIndex_z[3];
    real_T frac_10[3];
    uint32_T bpIndex_10[3];
    real_T frac_11[3];
    uint32_T bpIndex_11[3];
    real_T frac_12[3];
    uint32_T bpIndex_12[3];
    real_T frac_13[3];
    uint32_T bpIndex_13[3];
    real_T frac_14[3];
    uint32_T bpIndex_14[3];
    real_T frac_15[3];
    uint32_T bpIndex_15[3];
    real_T frac_16[3];
    uint32_T bpIndex_16[3];
    real_T frac_17[3];
    uint32_T bpIndex_17[3];
    real_T frac_18[3];
    uint32_T bpIndex_18[3];
    real_T frac_19[3];
    uint32_T bpIndex_19[3];
    real_T frac_1a[3];
    uint32_T bpIndex_1a[3];
    real_T frac_1b[3];
    uint32_T bpIndex_1b[3];
    real_T frac_1c[3];
    uint32_T bpIndex_1c[3];
    real_T frac_1d[3];
    uint32_T bpIndex_1d[3];
    real_T frac_1e[3];
    uint32_T bpIndex_1e[3];
    real_T frac_1f[3];
    uint32_T bpIndex_1f[3];
    real_T frac_1g[3];
    uint32_T bpIndex_1g[3];
    real_T frac_1h[3];
    uint32_T bpIndex_1h[3];
    real_T frac_1i[3];
    uint32_T bpIndex_1i[3];
    real_T frac_1j[3];
    uint32_T bpIndex_1j[3];
    real_T frac_1k[3];
    uint32_T bpIndex_1k[3];
    real_T frac_1l[3];
    uint32_T bpIndex_1l[3];
    real_T frac_1m[3];
    uint32_T bpIndex_1m[3];
    real_T frac_1n[3];
    uint32_T bpIndex_1n[3];
    real_T frac_1o[3];
    uint32_T bpIndex_1o[3];
    real_T frac_1p[3];
    uint32_T bpIndex_1p[3];
    real_T frac_1q[3];
    uint32_T bpIndex_1q[3];
    real_T frac_1r[3];
    uint32_T bpIndex_1r[3];
    real_T frac_1s[3];
    uint32_T bpIndex_1s[3];
    real_T frac_1t[3];
    uint32_T bpIndex_1t[3];
    real_T frac_1u[3];
    uint32_T bpIndex_1u[3];
    real_T frac_1v[3];
    uint32_T bpIndex_1v[3];
    real_T frac_1w[3];
    uint32_T bpIndex_1w[3];
    real_T frac_1x[3];
    uint32_T bpIndex_1x[3];
    real_T frac_1y[3];
    uint32_T bpIndex_1y[3];
    real_T frac_1z[3];
    uint32_T bpIndex_1z[3];
    real_T frac_20[3];
    uint32_T bpIndex_20[3];
    real_T frac_21[3];
    uint32_T bpIndex_21[3];
    real_T frac_22[3];
    uint32_T bpIndex_22[3];
    real_T frac_23[3];
    uint32_T bpIndex_23[3];
    real_T frac_24[3];
    uint32_T bpIndex_24[3];
    real_T frac_25[3];
    uint32_T bpIndex_25[3];
    real_T frac_26[3];
    uint32_T bpIndex_26[3];
    real_T frac_27[3];
    uint32_T bpIndex_27[3];
    real_T frac_28[3];
    uint32_T bpIndex_28[3];
    real_T frac_29[3];
    uint32_T bpIndex_29[3];
    real_T frac_2a[3];
    uint32_T bpIndex_2a[3];
    real_T frac_2b[3];
    uint32_T bpIndex_2b[3];
    real_T frac_2c[3];
    uint32_T bpIndex_2c[3];
    real_T frac_2d[3];
    uint32_T bpIndex_2d[3];
    real_T frac_2e[3];
    uint32_T bpIndex_2e[3];
    real_T frac_2f[3];
    uint32_T bpIndex_2f[3];
    real_T frac_2g[3];
    uint32_T bpIndex_2g[3];
    real_T frac_2h[3];
    uint32_T bpIndex_2h[3];
    real_T frac_2i[3];
    uint32_T bpIndex_2i[3];
    real_T frac_2j[3];
    uint32_T bpIndex_2j[3];
    real_T frac_2k[3];
    uint32_T bpIndex_2k[3];
    real_T frac_2l[3];
    uint32_T bpIndex_2l[3];
    real_T frac_2m[3];
    uint32_T bpIndex_2m[3];
    real_T frac_2n[3];
    uint32_T bpIndex_2n[3];
    real_T frac_2o[3];
    uint32_T bpIndex_2o[3];
    real_T frac_2p[3];
    uint32_T bpIndex_2p[3];
    real_T frac_2q[3];
    uint32_T bpIndex_2q[3];
    real_T frac_2r[3];
    uint32_T bpIndex_2r[3];
    real_T frac_2s[3];
    uint32_T bpIndex_2s[3];
    real_T frac_2t[3];
    uint32_T bpIndex_2t[3];
    real_T frac_2u[3];
    uint32_T bpIndex_2u[3];
    real_T frac_2v[3];
    uint32_T bpIndex_2v[3];
    real_T frac_2w[3];
    uint32_T bpIndex_2w[3];
    real_T frac_2x[3];
    uint32_T bpIndex_2x[3];
    real_T frac_2y[3];
    uint32_T bpIndex_2y[3];
    real_T frac_2z[3];
    uint32_T bpIndex_2z[3];
    real_T frac_30[3];
    uint32_T bpIndex_30[3];
    real_T frac_31[3];
    uint32_T bpIndex_31[3];
    real_T frac_32[3];
    uint32_T bpIndex_32[3];
    real_T frac_33[3];
    uint32_T bpIndex_33[3];
    real_T frac_34[3];
    uint32_T bpIndex_34[3];
    real_T frac_35[3];
    uint32_T bpIndex_35[3];
    real_T frac_36[3];
    uint32_T bpIndex_36[3];
    real_T frac_37[3];
    uint32_T bpIndex_37[3];
    real_T frac_38[3];
    uint32_T bpIndex_38[3];
    real_T frac_39[3];
    uint32_T bpIndex_39[3];
    real_T frac_3a[3];
    uint32_T bpIndex_3a[3];
    real_T frac_3b[3];
    uint32_T bpIndex_3b[3];
    real_T frac_3c[3];
    uint32_T bpIndex_3c[3];
    real_T frac_3d[3];
    uint32_T bpIndex_3d[3];
    real_T frac_3e[3];
    uint32_T bpIndex_3e[3];
    real_T frac_3f[3];
    uint32_T bpIndex_3f[3];
    real_T frac_3g[3];
    uint32_T bpIndex_3g[3];
    real_T frac_3h[3];
    uint32_T bpIndex_3h[3];
    real_T frac_3i[3];
    uint32_T bpIndex_3i[3];
    real_T frac_3j[3];
    uint32_T bpIndex_3j[3];
    real_T frac_3k[3];
    uint32_T bpIndex_3k[3];
    real_T frac_3l[3];
    uint32_T bpIndex_3l[3];
    real_T frac_3m[3];
    uint32_T bpIndex_3m[3];
    real_T frac_3n[3];
    uint32_T bpIndex_3n[3];
    real_T rtb_DCM_bs[9];
    real_T rtb_Transpose_tmp[9];
    real_T rtb_Sum_a_0[6];
    int32_T i;
    boolean_T rtb_RelationalOperator1;
    boolean_T rtb_RelationalOperator;
    boolean_T rtb_LowerRelop1;
    boolean_T rtb_RelationalOperator_c;
    boolean_T rtb_LowerRelop1_j;
    boolean_T rtb_RelationalOperator_l;
    boolean_T rtb_RelationalOperator_e_idx_1;
    boolean_T rtb_LowerRelop1_j_idx_1;
    boolean_T rtb_RelationalOperator_e_idx_0;
    boolean_T rtb_LowerRelop1_j_idx_0;
    boolean_T rtb_RelationalOperator_idx_1;
    boolean_T rtb_RelationalOperator1_idx_1;
    boolean_T rtb_RelationalOperator_idx_0;
    boolean_T rtb_RelationalOperator1_idx_0;
    real_T rtb_Product2_n_idx_1;
    real_T rtb_pgw_p_idx_0;
    boolean_T rtb_RelationalOperator_p_idx_1;
    boolean_T rtb_LowerRelop1_idx_1;
    boolean_T rtb_RelationalOperator_p_idx_0;
    boolean_T rtb_LowerRelop1_idx_0;
    real_T rtb_Product_d2_idx_4;
    real_T rtb_Product_d2_idx_5;
    real_T rtb_Product_ee_idx_0;
    real_T rtb_Product_ee_idx_1;
    real_T rtb_Product_ee_idx_2;
    real_T rtb_Product_ee_idx_3;
    real_T rtb_Product_ee_idx_4;
    real_T rtb_Product_ee_idx_5;
    real_T rtb_Divide_idx_0;
    real_T rtb_Divide_idx_1;
    real_T rtb_Divide_idx_2;
    real_T rtb_Divide_idx_3;
    real_T rtb_Divide_idx_4;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Saturate: '<S205>/Saturation' incorporates:
       *  DataStoreRead: '<S205>/Data Store Read'
       */
      if (Simulator_DW.CmdBus_e.thr > 1.0) {
        rtb_MediumHighAltitudeIntensity = 1.0;
      } else if (Simulator_DW.CmdBus_e.thr < -1.0) {
        rtb_MediumHighAltitudeIntensity = -1.0;
      } else {
        rtb_MediumHighAltitudeIntensity = Simulator_DW.CmdBus_e.thr;
      }

      /* End of Saturate: '<S205>/Saturation' */

      /* Fcn: '<S205>/Fcn1' */
      rtb_thr = (rtb_MediumHighAltitudeIntensity + 1.0) / 2.0;

      /* Saturate: '<S205>/Saturation1' incorporates:
       *  DataStoreRead: '<S205>/Data Store Read'
       */
      if (Simulator_DW.CmdBus_e.ail > 1.0) {
        rtb_MediumHighAltitudeIntensity = 1.0;
      } else if (Simulator_DW.CmdBus_e.ail < -1.0) {
        rtb_MediumHighAltitudeIntensity = -1.0;
      } else {
        rtb_MediumHighAltitudeIntensity = Simulator_DW.CmdBus_e.ail;
      }

      /* End of Saturate: '<S205>/Saturation1' */

      /* Gain: '<S205>/Gain' */
      rtb_ail = 45.0 * rtb_MediumHighAltitudeIntensity;

      /* Saturate: '<S205>/Saturation2' incorporates:
       *  DataStoreRead: '<S205>/Data Store Read'
       */
      if (Simulator_DW.CmdBus_e.elev > 1.0) {
        rtb_MediumHighAltitudeIntensity = 1.0;
      } else if (Simulator_DW.CmdBus_e.elev < -1.0) {
        rtb_MediumHighAltitudeIntensity = -1.0;
      } else {
        rtb_MediumHighAltitudeIntensity = Simulator_DW.CmdBus_e.elev;
      }

      /* End of Saturate: '<S205>/Saturation2' */

      /* Gain: '<S205>/Gain1' */
      rtb_elev = 45.0 * rtb_MediumHighAltitudeIntensity;

      /* Saturate: '<S205>/Saturation3' incorporates:
       *  DataStoreRead: '<S205>/Data Store Read'
       */
      if (Simulator_DW.CmdBus_e.rud > 1.0) {
        rtb_MediumHighAltitudeIntensity = 1.0;
      } else if (Simulator_DW.CmdBus_e.rud < -1.0) {
        rtb_MediumHighAltitudeIntensity = -1.0;
      } else {
        rtb_MediumHighAltitudeIntensity = Simulator_DW.CmdBus_e.rud;
      }

      /* End of Saturate: '<S205>/Saturation3' */

      /* Gain: '<S205>/Gain2' */
      rtb_rud = 45.0 * rtb_MediumHighAltitudeIntensity;

      /* Saturate: '<S205>/Saturation4' incorporates:
       *  DataStoreRead: '<S205>/Data Store Read'
       */
      if (Simulator_DW.CmdBus_e.flaps > 1.0) {
        rtb_MediumHighAltitudeIntensity = 1.0;
      } else if (Simulator_DW.CmdBus_e.flaps < -1.0) {
        rtb_MediumHighAltitudeIntensity = -1.0;
      } else {
        rtb_MediumHighAltitudeIntensity = Simulator_DW.CmdBus_e.flaps;
      }

      /* End of Saturate: '<S205>/Saturation4' */

      /* Gain: '<S205>/Gain4' */
      rtb_flaps = 45.0 * rtb_MediumHighAltitudeIntensity;

      /* Saturate: '<S205>/Saturation5' incorporates:
       *  DataStoreRead: '<S205>/Data Store Read'
       */
      if (Simulator_DW.CmdBus_e.ailOut > 1.0) {
        rtb_MediumHighAltitudeIntensity = 1.0;
      } else if (Simulator_DW.CmdBus_e.ailOut < -1.0) {
        rtb_MediumHighAltitudeIntensity = -1.0;
      } else {
        rtb_MediumHighAltitudeIntensity = Simulator_DW.CmdBus_e.ailOut;
      }

      /* End of Saturate: '<S205>/Saturation5' */

      /* Gain: '<S205>/Gain5' */
      rtb_ailOut = 45.0 * rtb_MediumHighAltitudeIntensity;

      /* Saturate: '<S205>/Saturation6' incorporates:
       *  DataStoreRead: '<S205>/Data Store Read'
       */
      if (Simulator_DW.CmdBus_e.thrDiff > 1.0) {
        rtb_thrDiff = 1.0;
      } else if (Simulator_DW.CmdBus_e.thrDiff < -1.0) {
        rtb_thrDiff = -1.0;
      } else {
        rtb_thrDiff = Simulator_DW.CmdBus_e.thrDiff;
      }

      /* End of Saturate: '<S205>/Saturation6' */

      /* Memory: '<S55>/Memory' */
      Simulator_B.Memory = Simulator_DW.Memory_PreviousInput;

      /* Logic: '<S55>/Logical Operator1' incorporates:
       *  Constant: '<S55>/Constant2'
       */
      Simulator_B.onGround = false;

      /* Memory: '<S8>/Memory' */
      rtb_platformStart = Simulator_DW.Memory_PreviousInput_f;

      /* Logic: '<S8>/NOT' incorporates:
       *  Memory: '<S8>/Memory'
       */
      Simulator_B.NOT = !Simulator_DW.Memory_PreviousInput_f;
    }

    /* Integrator: '<S205>/Integrator' */
    rtb_time = Simulator_X.Integrator_CSTATE;

    /* Integrator: '<S8>/Integrator' */
    if (rtmIsMajorTimeStep((&Simulator_M)) && Simulator_B.NOT) {
      /* evaluate the level of the reset signal */
      Simulator_X.Integrator_CSTATE_h = 0.0;
    }

    rtb_Saturation_m = Simulator_X.Integrator_CSTATE_h;

    /* Fcn: '<S56>/Fcn1' incorporates:
     *  Integrator: '<S8>/Integrator'
     */
    rtb_Fcn1 = (Simulator_X.Integrator_CSTATE_h < 6.0);

    /* Switch: '<S55>/Switch2' incorporates:
     *  Switch: '<S56>/Switch8'
     */
    if (Simulator_B.onGround) {
      rtb_reset[0] = true;
      rtb_reset[1] = true;
      rtb_reset[2] = false;
    } else if (rtb_Fcn1 > 0.0) {
      /* Switch: '<S56>/Switch8' */
      rtb_reset[0] = true;
      rtb_reset[1] = true;
      rtb_reset[2] = true;
    } else {
      rtb_reset[0] = false;
      rtb_reset[1] = false;
      rtb_reset[2] = false;
    }

    /* End of Switch: '<S55>/Switch2' */

    /* RelationalOperator: '<S45>/Relational Operator1' */
    rtb_RelationalOperator1 = (Simulator_X.Integrator_CSTATE_o[0] >= (rtInf));

    /* RelationalOperator: '<S45>/Relational Operator' */
    rtb_RelationalOperator = (Simulator_X.Integrator_CSTATE_o[0] <= (rtMinusInf));

    /* Logic: '<S45>/Logical Operator' */
    Simulator_B.LogicalOperator[0] = (rtb_RelationalOperator1 ||
      rtb_RelationalOperator);

    /* RelationalOperator: '<S45>/Relational Operator1' */
    rtb_RelationalOperator1_idx_0 = rtb_RelationalOperator1;

    /* RelationalOperator: '<S45>/Relational Operator' */
    rtb_RelationalOperator_idx_0 = rtb_RelationalOperator;

    /* RelationalOperator: '<S45>/Relational Operator1' */
    rtb_RelationalOperator1 = (Simulator_X.Integrator_CSTATE_o[1] >= (rtInf));

    /* RelationalOperator: '<S45>/Relational Operator' */
    rtb_RelationalOperator = (Simulator_X.Integrator_CSTATE_o[1] <= (rtMinusInf));

    /* Logic: '<S45>/Logical Operator' */
    Simulator_B.LogicalOperator[1] = (rtb_RelationalOperator1 ||
      rtb_RelationalOperator);

    /* RelationalOperator: '<S45>/Relational Operator1' */
    rtb_RelationalOperator1_idx_1 = rtb_RelationalOperator1;

    /* RelationalOperator: '<S45>/Relational Operator' */
    rtb_RelationalOperator_idx_1 = rtb_RelationalOperator;

    /* RelationalOperator: '<S45>/Relational Operator1' */
    rtb_RelationalOperator1 = (Simulator_X.Integrator_CSTATE_o[2] >= (rtInf));

    /* RelationalOperator: '<S45>/Relational Operator' */
    rtb_RelationalOperator = (Simulator_X.Integrator_CSTATE_o[2] <= (rtMinusInf));

    /* Logic: '<S45>/Logical Operator' */
    Simulator_B.LogicalOperator[2] = (rtb_RelationalOperator1 ||
      rtb_RelationalOperator);
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Memory: '<S48>/Memory' */
      Simulator_B.Memory_f[0] = Simulator_DW.Memory_PreviousInput_l[0];
      Simulator_B.Memory_f[1] = Simulator_DW.Memory_PreviousInput_l[1];
      Simulator_B.Memory_f[2] = Simulator_DW.Memory_PreviousInput_l[2];
    }

    /* Outputs for Enabled SubSystem: '<S48>/POSITIVE Edge' */
    Simulator_POSITIVEEdge(Simulator_ConstB.MultiportSwitch_i[0],
      Simulator_B.LogicalOperator, Simulator_B.Memory_f,
      &Simulator_B.POSITIVEEdge_o0, &Simulator_DW.POSITIVEEdge_o0);

    /* End of Outputs for SubSystem: '<S48>/POSITIVE Edge' */

    /* Outputs for Enabled SubSystem: '<S48>/NEGATIVE Edge' */
    Simulator_NEGATIVEEdge(Simulator_ConstB.MultiportSwitch_i[1],
      Simulator_B.LogicalOperator, Simulator_B.Memory_f,
      &Simulator_B.NEGATIVEEdge_j, &Simulator_DW.NEGATIVEEdge_j);

    /* End of Outputs for SubSystem: '<S48>/NEGATIVE Edge' */

    /* Logic: '<S48>/Logical Operator1' */
    Simulator_B.LogicalOperator1[0] =
      (Simulator_B.POSITIVEEdge_o0.RelationalOperator1[0] ||
       Simulator_B.NEGATIVEEdge_j.RelationalOperator1[0]);
    Simulator_B.LogicalOperator1[1] =
      (Simulator_B.POSITIVEEdge_o0.RelationalOperator1[1] ||
       Simulator_B.NEGATIVEEdge_j.RelationalOperator1[1]);
    Simulator_B.LogicalOperator1[2] =
      (Simulator_B.POSITIVEEdge_o0.RelationalOperator1[2] ||
       Simulator_B.NEGATIVEEdge_j.RelationalOperator1[2]);

    /* Logic: '<S45>/Logical Operator4' incorporates:
     *  Logic: '<S54>/Logical Operator4'
     */
    Simulator_B.LogicalOperator4[0] = (rtb_reset[0] ||
      Simulator_B.LogicalOperator1[0]);
    Simulator_B.LogicalOperator4[1] = (rtb_reset[1] ||
      Simulator_B.LogicalOperator1[1]);
    Simulator_B.LogicalOperator4[2] = (rtb_reset[2] ||
      Simulator_B.LogicalOperator1[2]);
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Switch: '<S55>/Switch3' */
      Simulator_B.Switch3[0] = 0.0;
      Simulator_B.Switch3[1] = 0.0;
      Simulator_B.Switch3[2] = 0.0;
    }

    /* Switch: '<S45>/Switch1' incorporates:
     *  Logic: '<S45>/Logical Operator5'
     *  Logic: '<S45>/Logical Operator6'
     */
    if (Simulator_B.LogicalOperator[0] && (!rtb_reset[0])) {
      /* Switch: '<S47>/Switch2' */
      Simulator_B.Switch2[0] = Simulator_X.Integrator_CSTATE_o[0];
    } else {
      /* Switch: '<S47>/Switch2' */
      Simulator_B.Switch2[0] = Simulator_B.Switch3[0];
    }

    if (Simulator_B.LogicalOperator[1] && (!rtb_reset[1])) {
      /* Switch: '<S47>/Switch2' */
      Simulator_B.Switch2[1] = Simulator_X.Integrator_CSTATE_o[1];
    } else {
      /* Switch: '<S47>/Switch2' */
      Simulator_B.Switch2[1] = Simulator_B.Switch3[1];
    }

    if (Simulator_B.LogicalOperator[2] && (!rtb_reset[2])) {
      /* Switch: '<S47>/Switch2' */
      Simulator_B.Switch2[2] = Simulator_X.Integrator_CSTATE_o[2];
    } else {
      /* Switch: '<S47>/Switch2' */
      Simulator_B.Switch2[2] = Simulator_B.Switch3[2];
    }

    /* End of Switch: '<S45>/Switch1' */

    /* Integrator: '<S45>/Integrator' */
    if (rtmIsMajorTimeStep((&Simulator_M))) {
      rEQ0 = false;
      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_m[0] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4[0])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_m[0] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_m[0] =
        Simulator_B.LogicalOperator4[0];
      if (Simulator_B.LogicalOperator4[0] || (Simulator_DW.Integrator_IWORK != 0))
      {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_o[0] = Simulator_B.Switch2[0];
        if (Simulator_X.Integrator_CSTATE_o[0] >= (rtInf)) {
          Simulator_X.Integrator_CSTATE_o[0] = (rtNaN);
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = didZcEventOccur;
      }

      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_m[1] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4[1])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_m[1] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_m[1] =
        Simulator_B.LogicalOperator4[1];
      if (Simulator_B.LogicalOperator4[1] || (Simulator_DW.Integrator_IWORK != 0))
      {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_o[1] = Simulator_B.Switch2[1];
        if (Simulator_X.Integrator_CSTATE_o[1] >= (rtInf)) {
          Simulator_X.Integrator_CSTATE_o[1] = (rtNaN);
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = (didZcEventOccur || rtb_LogicalOperator3_c);
      }

      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_m[2] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4[2])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_m[2] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_m[2] =
        Simulator_B.LogicalOperator4[2];
      if (Simulator_B.LogicalOperator4[2] || (Simulator_DW.Integrator_IWORK != 0))
      {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_o[2] = Simulator_B.Switch2[2];
        if (Simulator_X.Integrator_CSTATE_o[2] >= (rtInf)) {
          Simulator_X.Integrator_CSTATE_o[2] = (rtNaN);
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = (didZcEventOccur || rtb_LogicalOperator3_c);
      }

      if (rtb_LogicalOperator3_c) {
        rtsiSetBlockStateForSolverChangedAtMajorStep(&(&Simulator_M)->solverInfo,
          true);
        if (rEQ0) {
          rtsiSetContTimeOutputInconsistentWithStateAtMajorStep(&(&Simulator_M
            )->solverInfo, true);
        }
      }
    }

    rtb_Integrator_i[0] = Simulator_X.Integrator_CSTATE_o[0];
    rtb_Integrator_i[1] = Simulator_X.Integrator_CSTATE_o[1];
    rtb_Integrator_i[2] = Simulator_X.Integrator_CSTATE_o[2];

    /* End of Integrator: '<S45>/Integrator' */

    /* Switch: '<S55>/Switch7' incorporates:
     *  Switch: '<S56>/Switch9'
     */
    if (Simulator_B.onGround) {
      rtb_reset[0] = true;
      rtb_reset[1] = true;
      rtb_reset[2] = false;
    } else if (rtb_Fcn1 > 0.0) {
      /* Switch: '<S56>/Switch9' */
      rtb_reset[0] = true;
      rtb_reset[1] = true;
      rtb_reset[2] = true;
    } else {
      rtb_reset[0] = false;
      rtb_reset[1] = false;
      rtb_reset[2] = false;
    }

    /* End of Switch: '<S55>/Switch7' */

    /* RelationalOperator: '<S37>/Relational Operator1' */
    rtb_LowerRelop1 = (Simulator_X.Integrator_CSTATE_f[0] >= (rtInf));

    /* RelationalOperator: '<S37>/Relational Operator' */
    rtb_RelationalOperator_c = (Simulator_X.Integrator_CSTATE_f[0] <=
      (rtMinusInf));

    /* Logic: '<S37>/Logical Operator' */
    Simulator_B.LogicalOperator_c[0] = (rtb_LowerRelop1 ||
      rtb_RelationalOperator_c);

    /* RelationalOperator: '<S37>/Relational Operator1' */
    rtb_LowerRelop1_idx_0 = rtb_LowerRelop1;

    /* RelationalOperator: '<S37>/Relational Operator' */
    rtb_RelationalOperator_p_idx_0 = rtb_RelationalOperator_c;

    /* RelationalOperator: '<S37>/Relational Operator1' */
    rtb_LowerRelop1 = (Simulator_X.Integrator_CSTATE_f[1] >= (rtInf));

    /* RelationalOperator: '<S37>/Relational Operator' */
    rtb_RelationalOperator_c = (Simulator_X.Integrator_CSTATE_f[1] <=
      (rtMinusInf));

    /* Logic: '<S37>/Logical Operator' */
    Simulator_B.LogicalOperator_c[1] = (rtb_LowerRelop1 ||
      rtb_RelationalOperator_c);

    /* RelationalOperator: '<S37>/Relational Operator1' */
    rtb_LowerRelop1_idx_1 = rtb_LowerRelop1;

    /* RelationalOperator: '<S37>/Relational Operator' */
    rtb_RelationalOperator_p_idx_1 = rtb_RelationalOperator_c;

    /* RelationalOperator: '<S37>/Relational Operator1' */
    rtb_LowerRelop1 = (Simulator_X.Integrator_CSTATE_f[2] >= (rtInf));

    /* RelationalOperator: '<S37>/Relational Operator' */
    rtb_RelationalOperator_c = (Simulator_X.Integrator_CSTATE_f[2] <=
      (rtMinusInf));

    /* Logic: '<S37>/Logical Operator' */
    Simulator_B.LogicalOperator_c[2] = (rtb_LowerRelop1 ||
      rtb_RelationalOperator_c);
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Memory: '<S40>/Memory' */
      Simulator_B.Memory_j[0] = Simulator_DW.Memory_PreviousInput_a[0];
      Simulator_B.Memory_j[1] = Simulator_DW.Memory_PreviousInput_a[1];
      Simulator_B.Memory_j[2] = Simulator_DW.Memory_PreviousInput_a[2];
    }

    /* Outputs for Enabled SubSystem: '<S40>/POSITIVE Edge' */
    Simulator_POSITIVEEdge(Simulator_ConstB.MultiportSwitch_h[0],
      Simulator_B.LogicalOperator_c, Simulator_B.Memory_j,
      &Simulator_B.POSITIVEEdge_o, &Simulator_DW.POSITIVEEdge_o);

    /* End of Outputs for SubSystem: '<S40>/POSITIVE Edge' */

    /* Outputs for Enabled SubSystem: '<S40>/NEGATIVE Edge' */
    Simulator_NEGATIVEEdge(Simulator_ConstB.MultiportSwitch_h[1],
      Simulator_B.LogicalOperator_c, Simulator_B.Memory_j,
      &Simulator_B.NEGATIVEEdge_f, &Simulator_DW.NEGATIVEEdge_f);

    /* End of Outputs for SubSystem: '<S40>/NEGATIVE Edge' */

    /* Logic: '<S40>/Logical Operator1' */
    Simulator_B.LogicalOperator1_b[0] =
      (Simulator_B.POSITIVEEdge_o.RelationalOperator1[0] ||
       Simulator_B.NEGATIVEEdge_f.RelationalOperator1[0]);
    Simulator_B.LogicalOperator1_b[1] =
      (Simulator_B.POSITIVEEdge_o.RelationalOperator1[1] ||
       Simulator_B.NEGATIVEEdge_f.RelationalOperator1[1]);
    Simulator_B.LogicalOperator1_b[2] =
      (Simulator_B.POSITIVEEdge_o.RelationalOperator1[2] ||
       Simulator_B.NEGATIVEEdge_f.RelationalOperator1[2]);

    /* Logic: '<S37>/Logical Operator4' incorporates:
     *  Logic: '<S54>/Logical Operator7'
     */
    Simulator_B.LogicalOperator4_i[0] = (rtb_reset[0] ||
      Simulator_B.LogicalOperator1_b[0]);
    Simulator_B.LogicalOperator4_i[1] = (rtb_reset[1] ||
      Simulator_B.LogicalOperator1_b[1]);
    Simulator_B.LogicalOperator4_i[2] = (rtb_reset[2] ||
      Simulator_B.LogicalOperator1_b[2]);
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Switch: '<S55>/Switch8' */
      if (Simulator_B.onGround) {
        Simulator_B.Switch8[0] = 0.0;
        Simulator_B.Switch8[1] = 0.0;
        Simulator_B.Switch8[2] = 0.0;
      } else {
        Simulator_B.Switch8[0] = 0.0;
        Simulator_B.Switch8[1] = 0.0;
        Simulator_B.Switch8[2] = -0.122173;
      }

      /* End of Switch: '<S55>/Switch8' */
    }

    /* Switch: '<S37>/Switch1' incorporates:
     *  Logic: '<S37>/Logical Operator5'
     *  Logic: '<S37>/Logical Operator6'
     */
    if (Simulator_B.LogicalOperator_c[0] && (!rtb_reset[0])) {
      /* Switch: '<S39>/Switch2' */
      Simulator_B.Switch2_a[0] = Simulator_X.Integrator_CSTATE_f[0];
    } else {
      /* Switch: '<S39>/Switch2' */
      Simulator_B.Switch2_a[0] = Simulator_B.Switch8[0];
    }

    if (Simulator_B.LogicalOperator_c[1] && (!rtb_reset[1])) {
      /* Switch: '<S39>/Switch2' */
      Simulator_B.Switch2_a[1] = Simulator_X.Integrator_CSTATE_f[1];
    } else {
      /* Switch: '<S39>/Switch2' */
      Simulator_B.Switch2_a[1] = Simulator_B.Switch8[1];
    }

    if (Simulator_B.LogicalOperator_c[2] && (!rtb_reset[2])) {
      /* Switch: '<S39>/Switch2' */
      Simulator_B.Switch2_a[2] = Simulator_X.Integrator_CSTATE_f[2];
    } else {
      /* Switch: '<S39>/Switch2' */
      Simulator_B.Switch2_a[2] = Simulator_B.Switch8[2];
    }

    /* End of Switch: '<S37>/Switch1' */

    /* Integrator: '<S37>/Integrator' */
    if (rtmIsMajorTimeStep((&Simulator_M))) {
      rEQ0 = false;
      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_g[0] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4_i[0])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_g[0] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_g[0] =
        Simulator_B.LogicalOperator4_i[0];
      if (Simulator_B.LogicalOperator4_i[0] || (Simulator_DW.Integrator_IWORK_o
           != 0)) {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_f[0] = Simulator_B.Switch2_a[0];
        if ((Simulator_X.Integrator_CSTATE_f[0] < -3.1415926535897931) ||
            (Simulator_X.Integrator_CSTATE_f[0] >= 3.1415926535897931)) {
          rtb_InterpolationUsingPrelookup = std::floor
            ((Simulator_X.Integrator_CSTATE_f[0] - -3.1415926535897931) /
             6.2831853071795862);
          if (rtb_InterpolationUsingPrelookup != 0.0) {
            Simulator_X.Integrator_CSTATE_f[0] =
              ((Simulator_X.Integrator_CSTATE_f[0] - -3.1415926535897931) -
               rtb_InterpolationUsingPrelookup * 6.2831853071795862) +
              -3.1415926535897931;
          }
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = didZcEventOccur;
      }

      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_g[1] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4_i[1])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_g[1] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_g[1] =
        Simulator_B.LogicalOperator4_i[1];
      if (Simulator_B.LogicalOperator4_i[1] || (Simulator_DW.Integrator_IWORK_o
           != 0)) {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_f[1] = Simulator_B.Switch2_a[1];
        if ((Simulator_X.Integrator_CSTATE_f[1] < -3.1415926535897931) ||
            (Simulator_X.Integrator_CSTATE_f[1] >= 3.1415926535897931)) {
          rtb_InterpolationUsingPrelookup = std::floor
            ((Simulator_X.Integrator_CSTATE_f[1] - -3.1415926535897931) /
             6.2831853071795862);
          if (rtb_InterpolationUsingPrelookup != 0.0) {
            Simulator_X.Integrator_CSTATE_f[1] =
              ((Simulator_X.Integrator_CSTATE_f[1] - -3.1415926535897931) -
               rtb_InterpolationUsingPrelookup * 6.2831853071795862) +
              -3.1415926535897931;
          }
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = (didZcEventOccur || rtb_LogicalOperator3_c);
      }

      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_g[2] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4_i[2])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_g[2] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_g[2] =
        Simulator_B.LogicalOperator4_i[2];
      if (Simulator_B.LogicalOperator4_i[2] || (Simulator_DW.Integrator_IWORK_o
           != 0)) {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_f[2] = Simulator_B.Switch2_a[2];
        if ((Simulator_X.Integrator_CSTATE_f[2] < -3.1415926535897931) ||
            (Simulator_X.Integrator_CSTATE_f[2] >= 3.1415926535897931)) {
          rtb_InterpolationUsingPrelookup = std::floor
            ((Simulator_X.Integrator_CSTATE_f[2] - -3.1415926535897931) /
             6.2831853071795862);
          if (rtb_InterpolationUsingPrelookup != 0.0) {
            Simulator_X.Integrator_CSTATE_f[2] =
              ((Simulator_X.Integrator_CSTATE_f[2] - -3.1415926535897931) -
               rtb_InterpolationUsingPrelookup * 6.2831853071795862) +
              -3.1415926535897931;
          }
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = (didZcEventOccur || rtb_LogicalOperator3_c);
      }

      if (rtb_LogicalOperator3_c) {
        rtsiSetBlockStateForSolverChangedAtMajorStep(&(&Simulator_M)->solverInfo,
          true);
        if (rEQ0) {
          rtsiSetContTimeOutputInconsistentWithStateAtMajorStep(&(&Simulator_M
            )->solverInfo, true);
        }
      }
    }

    rtb_sincos_o1[0] = Simulator_X.Integrator_CSTATE_f[0];

    /* Trigonometry: '<S33>/sincos' incorporates:
     *  Integrator: '<S37>/Integrator'
     *  SignalConversion generated from: '<S33>/sincos'
     */
    rtb_Mstabtobody[0] = std::cos(Simulator_X.Integrator_CSTATE_f[2]);
    rtb_Product_ly[0] = std::sin(Simulator_X.Integrator_CSTATE_f[2]);

    /* Integrator: '<S37>/Integrator' */
    rtb_sincos_o1[1] = Simulator_X.Integrator_CSTATE_f[1];

    /* Trigonometry: '<S33>/sincos' incorporates:
     *  Integrator: '<S37>/Integrator'
     *  SignalConversion generated from: '<S33>/sincos'
     */
    rtb_Mstabtobody[1] = std::cos(Simulator_X.Integrator_CSTATE_f[1]);
    rtb_Product_ly[1] = std::sin(Simulator_X.Integrator_CSTATE_f[1]);

    /* Integrator: '<S37>/Integrator' */
    rtb_sincos_o1[2] = Simulator_X.Integrator_CSTATE_f[2];

    /* Trigonometry: '<S33>/sincos' incorporates:
     *  Integrator: '<S37>/Integrator'
     *  SignalConversion generated from: '<S33>/sincos'
     */
    rtb_Mstabtobody[2] = std::cos(Simulator_X.Integrator_CSTATE_f[0]);
    rtb_InterpolationUsingPrelookup = std::sin(Simulator_X.Integrator_CSTATE_f[0]);

    /* Fcn: '<S33>/Fcn11' */
    Simulator_B.VectorConcatenate[0] = rtb_Mstabtobody[1] * rtb_Mstabtobody[0];

    /* Fcn: '<S33>/Fcn21' incorporates:
     *  Fcn: '<S33>/Fcn22'
     */
    rtb_UnitConversion = rtb_InterpolationUsingPrelookup * rtb_Product_ly[1];
    Simulator_B.VectorConcatenate[1] = rtb_UnitConversion * rtb_Mstabtobody[0] -
      rtb_Mstabtobody[2] * rtb_Product_ly[0];

    /* Fcn: '<S33>/Fcn31' incorporates:
     *  Fcn: '<S33>/Fcn32'
     */
    rtb_UnitConversion_i = rtb_Mstabtobody[2] * rtb_Product_ly[1];
    Simulator_B.VectorConcatenate[2] = rtb_UnitConversion_i * rtb_Mstabtobody[0]
      + rtb_InterpolationUsingPrelookup * rtb_Product_ly[0];

    /* Fcn: '<S33>/Fcn12' */
    Simulator_B.VectorConcatenate[3] = rtb_Mstabtobody[1] * rtb_Product_ly[0];

    /* Fcn: '<S33>/Fcn22' */
    Simulator_B.VectorConcatenate[4] = rtb_UnitConversion * rtb_Product_ly[0] +
      rtb_Mstabtobody[2] * rtb_Mstabtobody[0];

    /* Fcn: '<S33>/Fcn32' */
    Simulator_B.VectorConcatenate[5] = rtb_UnitConversion_i * rtb_Product_ly[0]
      - rtb_InterpolationUsingPrelookup * rtb_Mstabtobody[0];

    /* Fcn: '<S33>/Fcn13' */
    Simulator_B.VectorConcatenate[6] = -rtb_Product_ly[1];

    /* Fcn: '<S33>/Fcn23' */
    Simulator_B.VectorConcatenate[7] = rtb_InterpolationUsingPrelookup *
      rtb_Mstabtobody[1];

    /* Fcn: '<S33>/Fcn33' */
    Simulator_B.VectorConcatenate[8] = rtb_Mstabtobody[2] * rtb_Mstabtobody[1];
    for (i = 0; i < 3; i++) {
      /* Math: '<S6>/Transpose' */
      rtb_Transpose_tmp[3 * i] = Simulator_B.VectorConcatenate[i];
      rtb_Transpose_tmp[3 * i + 1] = Simulator_B.VectorConcatenate[i + 3];
      rtb_Transpose_tmp[3 * i + 2] = Simulator_B.VectorConcatenate[i + 6];

      /* Logic: '<S54>/Logical Operator3' incorporates:
       *  Switch: '<S56>/Switch3'
       */
      rtb_reset[i] = (rtb_Fcn1 > 0.0);
    }

    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Switch: '<S55>/Switch1' */
      if (Simulator_B.onGround) {
        Simulator_B.Switch1[0] = (rtInf);
        Simulator_B.Switch1[1] = (rtInf);
        Simulator_B.Switch1[2] = 0.0;
      } else {
        Simulator_B.Switch1[0] = (rtInf);
        Simulator_B.Switch1[1] = (rtInf);
        Simulator_B.Switch1[2] = (rtInf);
      }

      /* End of Switch: '<S55>/Switch1' */

      /* Memory: '<S23>/Memory' */
      Simulator_B.Memory_d[0] = Simulator_DW.Memory_PreviousInput_a5[0];
      Simulator_B.Memory_d[1] = Simulator_DW.Memory_PreviousInput_a5[1];
      Simulator_B.Memory_d[2] = Simulator_DW.Memory_PreviousInput_a5[2];
    }

    /* RelationalOperator: '<S20>/Relational Operator1' */
    rEQ0 = (Simulator_X.Integrator_CSTATE_b[0] >= Simulator_B.Switch1[0]);

    /* RelationalOperator: '<S20>/Relational Operator' */
    didZcEventOccur = (Simulator_X.Integrator_CSTATE_b[0] <= (rtMinusInf));

    /* Logic: '<S20>/Logical Operator' */
    Simulator_B.LogicalOperator_b[0] = (rEQ0 || didZcEventOccur);

    /* RelationalOperator: '<S20>/Relational Operator1' */
    rtb_LowerRelop1_g[0] = rEQ0;

    /* RelationalOperator: '<S20>/Relational Operator' */
    rtb_LogicalOperator3_d[0] = didZcEventOccur;

    /* RelationalOperator: '<S20>/Relational Operator1' */
    rEQ0 = (Simulator_X.Integrator_CSTATE_b[1] >= Simulator_B.Switch1[1]);

    /* RelationalOperator: '<S20>/Relational Operator' */
    didZcEventOccur = (Simulator_X.Integrator_CSTATE_b[1] <= (rtMinusInf));

    /* Logic: '<S20>/Logical Operator' */
    Simulator_B.LogicalOperator_b[1] = (rEQ0 || didZcEventOccur);

    /* RelationalOperator: '<S20>/Relational Operator1' */
    rtb_LowerRelop1_g[1] = rEQ0;

    /* RelationalOperator: '<S20>/Relational Operator' */
    rtb_LogicalOperator3_d[1] = didZcEventOccur;

    /* RelationalOperator: '<S20>/Relational Operator1' */
    rEQ0 = (Simulator_X.Integrator_CSTATE_b[2] >= Simulator_B.Switch1[2]);

    /* RelationalOperator: '<S20>/Relational Operator' */
    didZcEventOccur = (Simulator_X.Integrator_CSTATE_b[2] <= (rtMinusInf));

    /* Logic: '<S20>/Logical Operator' */
    Simulator_B.LogicalOperator_b[2] = (rEQ0 || didZcEventOccur);

    /* RelationalOperator: '<S20>/Relational Operator1' */
    rtb_LowerRelop1_g[2] = rEQ0;

    /* RelationalOperator: '<S20>/Relational Operator' */
    rtb_LogicalOperator3_d[2] = didZcEventOccur;

    /* Outputs for Enabled SubSystem: '<S23>/POSITIVE Edge' */
    Simulator_POSITIVEEdge(Simulator_ConstB.MultiportSwitch[0],
      Simulator_B.LogicalOperator_b, Simulator_B.Memory_d,
      &Simulator_B.POSITIVEEdge, &Simulator_DW.POSITIVEEdge);

    /* End of Outputs for SubSystem: '<S23>/POSITIVE Edge' */

    /* Outputs for Enabled SubSystem: '<S23>/NEGATIVE Edge' */
    Simulator_NEGATIVEEdge(Simulator_ConstB.MultiportSwitch[1],
      Simulator_B.LogicalOperator_b, Simulator_B.Memory_d,
      &Simulator_B.NEGATIVEEdge, &Simulator_DW.NEGATIVEEdge);

    /* End of Outputs for SubSystem: '<S23>/NEGATIVE Edge' */

    /* Logic: '<S23>/Logical Operator1' */
    Simulator_B.LogicalOperator1_k[0] =
      (Simulator_B.POSITIVEEdge.RelationalOperator1[0] ||
       Simulator_B.NEGATIVEEdge.RelationalOperator1[0]);
    Simulator_B.LogicalOperator1_k[1] =
      (Simulator_B.POSITIVEEdge.RelationalOperator1[1] ||
       Simulator_B.NEGATIVEEdge.RelationalOperator1[1]);
    Simulator_B.LogicalOperator1_k[2] =
      (Simulator_B.POSITIVEEdge.RelationalOperator1[2] ||
       Simulator_B.NEGATIVEEdge.RelationalOperator1[2]);

    /* Logic: '<S20>/Logical Operator4' */
    Simulator_B.LogicalOperator4_d[0] = (rtb_reset[0] ||
      Simulator_B.LogicalOperator1_k[0]);
    Simulator_B.LogicalOperator4_d[1] = (rtb_reset[1] ||
      Simulator_B.LogicalOperator1_k[1]);
    Simulator_B.LogicalOperator4_d[2] = (rtb_reset[2] ||
      Simulator_B.LogicalOperator1_k[2]);

    /* Lookup_n-D: '<S8>/1-D Lookup Table' */
    rtb_InterpolationUsingPrelookup = look1_binlc(rtb_Saturation_m,
      Simulator_ConstP.pooled9, Simulator_ConstP.uDLookupTable_tableData, 1U);

    /* Gain: '<S8>/cos(phi)' */
    Simulator_B.Vx = 0.992546157447128 * rtb_InterpolationUsingPrelookup;

    /* Gain: '<S8>/sin(phi)' */
    Simulator_B.Vy = -0.12186929612064265 * rtb_InterpolationUsingPrelookup;

    /* Gain: '<S8>/Gain' incorporates:
     *  Fcn: '<S8>/Fcn1'
     */
    Simulator_B.Vz = static_cast<real_T>(((rtb_Saturation_m > 0.0) &&
      (rtb_Saturation_m < 1.0))) * -10.0;

    /* Switch: '<S56>/Switch' incorporates:
     *  Product: '<S56>/MatrixMultiply'
     *  SignalConversion generated from: '<S56>/MatrixMultiply'
     */
    if (rtb_Fcn1 > 0.0) {
      for (i = 0; i < 3; i++) {
        rtb_Mstabtobody[i] = Simulator_B.VectorConcatenate[i + 6] *
          Simulator_B.Vz + (Simulator_B.VectorConcatenate[i + 3] *
                            Simulator_B.Vy + Simulator_B.VectorConcatenate[i] *
                            Simulator_B.Vx);
      }
    } else {
      rtb_Mstabtobody[0] = 0.0;
      rtb_Mstabtobody[1] = 0.0;
      rtb_Mstabtobody[2] = 0.0;
    }

    /* End of Switch: '<S56>/Switch' */

    /* Switch: '<S20>/Switch1' incorporates:
     *  Logic: '<S20>/Logical Operator5'
     *  Logic: '<S20>/Logical Operator6'
     */
    rtb_Saturation_m = rtb_Mstabtobody[0];
    if (Simulator_B.LogicalOperator_b[0] && (!rtb_reset[0])) {
      rtb_Saturation_m = Simulator_X.Integrator_CSTATE_b[0];
    }

    /* Switch: '<S22>/Switch2' incorporates:
     *  RelationalOperator: '<S22>/LowerRelop1'
     */
    if (rtb_Saturation_m > Simulator_B.Switch1[0]) {
      Simulator_B.Switch2_n[0] = Simulator_B.Switch1[0];
    } else {
      Simulator_B.Switch2_n[0] = rtb_Saturation_m;
    }

    /* Switch: '<S20>/Switch1' incorporates:
     *  Logic: '<S20>/Logical Operator5'
     *  Logic: '<S20>/Logical Operator6'
     */
    rtb_Saturation_m = rtb_Mstabtobody[1];
    if (Simulator_B.LogicalOperator_b[1] && (!rtb_reset[1])) {
      rtb_Saturation_m = Simulator_X.Integrator_CSTATE_b[1];
    }

    /* Switch: '<S22>/Switch2' incorporates:
     *  RelationalOperator: '<S22>/LowerRelop1'
     */
    if (rtb_Saturation_m > Simulator_B.Switch1[1]) {
      Simulator_B.Switch2_n[1] = Simulator_B.Switch1[1];
    } else {
      Simulator_B.Switch2_n[1] = rtb_Saturation_m;
    }

    /* Switch: '<S20>/Switch1' incorporates:
     *  Logic: '<S20>/Logical Operator5'
     *  Logic: '<S20>/Logical Operator6'
     */
    rtb_Saturation_m = rtb_Mstabtobody[2];
    if (Simulator_B.LogicalOperator_b[2] && (!rtb_reset[2])) {
      rtb_Saturation_m = Simulator_X.Integrator_CSTATE_b[2];
    }

    /* Switch: '<S22>/Switch2' incorporates:
     *  RelationalOperator: '<S22>/LowerRelop1'
     */
    if (rtb_Saturation_m > Simulator_B.Switch1[2]) {
      Simulator_B.Switch2_n[2] = Simulator_B.Switch1[2];
    } else {
      Simulator_B.Switch2_n[2] = rtb_Saturation_m;
    }

    /* Integrator: '<S20>/Integrator' */
    if (rtmIsMajorTimeStep((&Simulator_M))) {
      rEQ0 = false;
      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_my[0] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4_d[0])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_my[0] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_my[0] =
        Simulator_B.LogicalOperator4_d[0];
      if (Simulator_B.LogicalOperator4_d[0] || (Simulator_DW.Integrator_IWORK_p
           != 0)) {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_b[0] = Simulator_B.Switch2_n[0];
        if (Simulator_X.Integrator_CSTATE_b[0] >= (rtInf)) {
          Simulator_X.Integrator_CSTATE_b[0] = (rtNaN);
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = didZcEventOccur;
      }

      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_my[1] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4_d[1])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_my[1] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_my[1] =
        Simulator_B.LogicalOperator4_d[1];
      if (Simulator_B.LogicalOperator4_d[1] || (Simulator_DW.Integrator_IWORK_p
           != 0)) {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_b[1] = Simulator_B.Switch2_n[1];
        if (Simulator_X.Integrator_CSTATE_b[1] >= (rtInf)) {
          Simulator_X.Integrator_CSTATE_b[1] = (rtNaN);
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = (didZcEventOccur || rtb_LogicalOperator3_c);
      }

      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_my[2] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4_d[2])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_my[2] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_my[2] =
        Simulator_B.LogicalOperator4_d[2];
      if (Simulator_B.LogicalOperator4_d[2] || (Simulator_DW.Integrator_IWORK_p
           != 0)) {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_b[2] = Simulator_B.Switch2_n[2];
        if (Simulator_X.Integrator_CSTATE_b[2] >= (rtInf)) {
          Simulator_X.Integrator_CSTATE_b[2] = (rtNaN);
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = (didZcEventOccur || rtb_LogicalOperator3_c);
      }

      if (rtb_LogicalOperator3_c) {
        rtsiSetBlockStateForSolverChangedAtMajorStep(&(&Simulator_M)->solverInfo,
          true);
        if (rEQ0) {
          rtsiSetContTimeOutputInconsistentWithStateAtMajorStep(&(&Simulator_M
            )->solverInfo, true);
        }
      }
    }

    /* Logic: '<S6>/NOT' incorporates:
     *  Logic: '<S56>/Logical Operator3'
     */
    rtb_LogicalOperator2_p = !(rtb_Fcn1 != 0.0);
    for (i = 0; i < 3; i++) {
      /* Integrator: '<S20>/Integrator' */
      rtb_Mstabtobody[i] = Simulator_X.Integrator_CSTATE_b[i];

      /* Logic: '<S54>/Logical Operator5' incorporates:
       *  Switch: '<S56>/Switch4'
       */
      rtb_reset[i] = (rtb_Fcn1 > 0.0);

      /* Product: '<S19>/Product' incorporates:
       *  Integrator: '<S20>/Integrator'
       *  Math: '<S6>/Transpose'
       */
      rtb_Product_ly[i] = rtb_Transpose_tmp[i + 6] *
        Simulator_X.Integrator_CSTATE_b[2] + (rtb_Transpose_tmp[i + 3] *
        Simulator_X.Integrator_CSTATE_b[1] + rtb_Transpose_tmp[i] *
        Simulator_X.Integrator_CSTATE_b[0]);
    }

    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Switch: '<S55>/Switch4' */
      if (Simulator_B.onGround) {
        Simulator_B.Switch4[0] = (rtInf);
        Simulator_B.Switch4[1] = (rtInf);
        Simulator_B.Switch4[2] = 0.0;
      } else {
        Simulator_B.Switch4[0] = (rtInf);
        Simulator_B.Switch4[1] = (rtInf);
        Simulator_B.Switch4[2] = (rtInf);
      }

      /* End of Switch: '<S55>/Switch4' */

      /* Memory: '<S63>/Memory' */
      Simulator_B.Memory_n[0] = Simulator_DW.Memory_PreviousInput_n[0];
      Simulator_B.Memory_n[1] = Simulator_DW.Memory_PreviousInput_n[1];
      Simulator_B.Memory_n[2] = Simulator_DW.Memory_PreviousInput_n[2];
    }

    /* RelationalOperator: '<S60>/Relational Operator1' */
    rtb_LowerRelop1_j = (Simulator_X.Integrator_CSTATE_a[0] >=
                         Simulator_B.Switch4[0]);

    /* RelationalOperator: '<S60>/Relational Operator' */
    rtb_RelationalOperator_l = (Simulator_X.Integrator_CSTATE_a[0] <=
      (rtMinusInf));

    /* Logic: '<S60>/Logical Operator' */
    Simulator_B.LogicalOperator_o[0] = (rtb_LowerRelop1_j ||
      rtb_RelationalOperator_l);

    /* RelationalOperator: '<S60>/Relational Operator1' */
    rtb_LowerRelop1_j_idx_0 = rtb_LowerRelop1_j;

    /* RelationalOperator: '<S60>/Relational Operator' */
    rtb_RelationalOperator_e_idx_0 = rtb_RelationalOperator_l;

    /* RelationalOperator: '<S60>/Relational Operator1' */
    rtb_LowerRelop1_j = (Simulator_X.Integrator_CSTATE_a[1] >=
                         Simulator_B.Switch4[1]);

    /* RelationalOperator: '<S60>/Relational Operator' */
    rtb_RelationalOperator_l = (Simulator_X.Integrator_CSTATE_a[1] <=
      (rtMinusInf));

    /* Logic: '<S60>/Logical Operator' */
    Simulator_B.LogicalOperator_o[1] = (rtb_LowerRelop1_j ||
      rtb_RelationalOperator_l);

    /* RelationalOperator: '<S60>/Relational Operator1' */
    rtb_LowerRelop1_j_idx_1 = rtb_LowerRelop1_j;

    /* RelationalOperator: '<S60>/Relational Operator' */
    rtb_RelationalOperator_e_idx_1 = rtb_RelationalOperator_l;

    /* RelationalOperator: '<S60>/Relational Operator1' */
    rtb_LowerRelop1_j = (Simulator_X.Integrator_CSTATE_a[2] >=
                         Simulator_B.Switch4[2]);

    /* RelationalOperator: '<S60>/Relational Operator' */
    rtb_RelationalOperator_l = (Simulator_X.Integrator_CSTATE_a[2] <=
      (rtMinusInf));

    /* Logic: '<S60>/Logical Operator' */
    Simulator_B.LogicalOperator_o[2] = (rtb_LowerRelop1_j ||
      rtb_RelationalOperator_l);

    /* Outputs for Enabled SubSystem: '<S63>/POSITIVE Edge' */
    Simulator_POSITIVEEdge(Simulator_ConstB.MultiportSwitch_ib[0],
      Simulator_B.LogicalOperator_o, Simulator_B.Memory_n,
      &Simulator_B.POSITIVEEdge_l, &Simulator_DW.POSITIVEEdge_l);

    /* End of Outputs for SubSystem: '<S63>/POSITIVE Edge' */

    /* Outputs for Enabled SubSystem: '<S63>/NEGATIVE Edge' */
    Simulator_NEGATIVEEdge(Simulator_ConstB.MultiportSwitch_ib[1],
      Simulator_B.LogicalOperator_o, Simulator_B.Memory_n,
      &Simulator_B.NEGATIVEEdge_fy, &Simulator_DW.NEGATIVEEdge_fy);

    /* End of Outputs for SubSystem: '<S63>/NEGATIVE Edge' */

    /* Logic: '<S63>/Logical Operator1' */
    Simulator_B.LogicalOperator1_j[0] =
      (Simulator_B.POSITIVEEdge_l.RelationalOperator1[0] ||
       Simulator_B.NEGATIVEEdge_fy.RelationalOperator1[0]);
    Simulator_B.LogicalOperator1_j[1] =
      (Simulator_B.POSITIVEEdge_l.RelationalOperator1[1] ||
       Simulator_B.NEGATIVEEdge_fy.RelationalOperator1[1]);
    Simulator_B.LogicalOperator1_j[2] =
      (Simulator_B.POSITIVEEdge_l.RelationalOperator1[2] ||
       Simulator_B.NEGATIVEEdge_fy.RelationalOperator1[2]);

    /* Switch: '<S60>/Switch1' incorporates:
     *  Logic: '<S60>/Logical Operator5'
     *  Logic: '<S60>/Logical Operator6'
     *  Switch: '<S56>/Switch1'
     */
    if (Simulator_B.LogicalOperator_o[0] && (!rtb_reset[0])) {
      rtb_Integrator_o[0] = Simulator_X.Integrator_CSTATE_a[0];
    } else if (rtb_Fcn1 > 0.0) {
      /* Switch: '<S56>/Switch1' incorporates:
       *  Integrator: '<S8>/Integrator2'
       */
      rtb_Integrator_o[0] = Simulator_X.Integrator2_CSTATE;
    } else {
      rtb_Integrator_o[0] = 0.0;
    }

    if (Simulator_B.LogicalOperator_o[1] && (!rtb_reset[1])) {
      rtb_Integrator_o[1] = Simulator_X.Integrator_CSTATE_a[1];
    } else if (rtb_Fcn1 > 0.0) {
      /* Switch: '<S56>/Switch1' incorporates:
       *  Integrator: '<S8>/Integrator1'
       */
      rtb_Integrator_o[1] = Simulator_X.Integrator1_CSTATE;
    } else {
      rtb_Integrator_o[1] = 0.0;
    }

    if (Simulator_B.LogicalOperator_o[2] && (!rtb_reset[2])) {
      rtb_Integrator_o[2] = Simulator_X.Integrator_CSTATE_a[2];
    } else if (rtb_Fcn1 > 0.0) {
      /* Switch: '<S56>/Switch1' incorporates:
       *  Integrator: '<S8>/Integrator3'
       */
      rtb_Integrator_o[2] = Simulator_X.Integrator3_CSTATE;
    } else {
      rtb_Integrator_o[2] = 0.0;
    }

    /* End of Switch: '<S60>/Switch1' */

    /* Logic: '<S60>/Logical Operator4' */
    Simulator_B.LogicalOperator4_a[0] = (rtb_reset[0] ||
      Simulator_B.LogicalOperator1_j[0]);

    /* Switch: '<S62>/Switch2' incorporates:
     *  RelationalOperator: '<S62>/LowerRelop1'
     */
    if (rtb_Integrator_o[0] > Simulator_B.Switch4[0]) {
      Simulator_B.Switch2_f[0] = Simulator_B.Switch4[0];
    } else {
      Simulator_B.Switch2_f[0] = rtb_Integrator_o[0];
    }

    /* Logic: '<S60>/Logical Operator4' */
    Simulator_B.LogicalOperator4_a[1] = (rtb_reset[1] ||
      Simulator_B.LogicalOperator1_j[1]);

    /* Switch: '<S62>/Switch2' incorporates:
     *  RelationalOperator: '<S62>/LowerRelop1'
     */
    if (rtb_Integrator_o[1] > Simulator_B.Switch4[1]) {
      Simulator_B.Switch2_f[1] = Simulator_B.Switch4[1];
    } else {
      Simulator_B.Switch2_f[1] = rtb_Integrator_o[1];
    }

    /* Logic: '<S60>/Logical Operator4' */
    Simulator_B.LogicalOperator4_a[2] = (rtb_reset[2] ||
      Simulator_B.LogicalOperator1_j[2]);

    /* Switch: '<S62>/Switch2' incorporates:
     *  RelationalOperator: '<S62>/LowerRelop1'
     */
    if (rtb_Integrator_o[2] > Simulator_B.Switch4[2]) {
      Simulator_B.Switch2_f[2] = Simulator_B.Switch4[2];
    } else {
      Simulator_B.Switch2_f[2] = rtb_Integrator_o[2];
    }

    /* Integrator: '<S60>/Integrator' */
    if (rtmIsMajorTimeStep((&Simulator_M))) {
      rEQ0 = false;
      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_c[0] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4_a[0])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_c[0] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_c[0] =
        Simulator_B.LogicalOperator4_a[0];
      if (Simulator_B.LogicalOperator4_a[0] || (Simulator_DW.Integrator_IWORK_c
           != 0)) {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_a[0] = Simulator_B.Switch2_f[0];
        if (Simulator_X.Integrator_CSTATE_a[0] >= (rtInf)) {
          Simulator_X.Integrator_CSTATE_a[0] = (rtNaN);
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = didZcEventOccur;
      }

      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_c[1] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4_a[1])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_c[1] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_c[1] =
        Simulator_B.LogicalOperator4_a[1];
      if (Simulator_B.LogicalOperator4_a[1] || (Simulator_DW.Integrator_IWORK_c
           != 0)) {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_a[1] = Simulator_B.Switch2_f[1];
        if (Simulator_X.Integrator_CSTATE_a[1] >= (rtInf)) {
          Simulator_X.Integrator_CSTATE_a[1] = (rtNaN);
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = (didZcEventOccur || rtb_LogicalOperator3_c);
      }

      didZcEventOccur = (((Simulator_PrevZCX.Integrator_Reset_ZCE_c[2] ==
                           POS_ZCSIG) != static_cast<int32_T>
                          (Simulator_B.LogicalOperator4_a[2])) &&
                         (Simulator_PrevZCX.Integrator_Reset_ZCE_c[2] !=
                          UNINITIALIZED_ZCSIG));
      Simulator_PrevZCX.Integrator_Reset_ZCE_c[2] =
        Simulator_B.LogicalOperator4_a[2];
      if (Simulator_B.LogicalOperator4_a[2] || (Simulator_DW.Integrator_IWORK_c
           != 0)) {
        rtb_LogicalOperator3_c = true;
        Simulator_X.Integrator_CSTATE_a[2] = Simulator_B.Switch2_f[2];
        if (Simulator_X.Integrator_CSTATE_a[2] >= (rtInf)) {
          Simulator_X.Integrator_CSTATE_a[2] = (rtNaN);
        }

        rEQ0 = true;
      } else {
        rtb_LogicalOperator3_c = (didZcEventOccur || rtb_LogicalOperator3_c);
      }

      if (rtb_LogicalOperator3_c) {
        rtsiSetBlockStateForSolverChangedAtMajorStep(&(&Simulator_M)->solverInfo,
          true);
        if (rEQ0) {
          rtsiSetContTimeOutputInconsistentWithStateAtMajorStep(&(&Simulator_M
            )->solverInfo, true);
        }
      }
    }

    /* Sum: '<S245>/Add' incorporates:
     *  Gain: '<S9>/xyH'
     *  Integrator: '<S60>/Integrator'
     */
    Simulator_B.Add[0] = 37.684341;
    Simulator_B.Add[1] = -121.436448;
    Simulator_B.Add[2] = -Simulator_X.Integrator_CSTATE_a[2];

    /* S-Function (saeroatmos): '<S247>/S-Function' */
    {
      /* S-Function Block: <S247>/S-Function */
      real_T *temp_table = (real_T *) &Simulator_DW.SFunction_temp_table[0];
      real_T *pres_table = (real_T *) &Simulator_DW.SFunction_pres_table[0];

      /* COESA */
      CalcAtmosCOESA( &Simulator_B.Add[2], &Simulator_B.SFunction_o1,
                     &Simulator_B.SFunction_o3, &Simulator_B.SFunction_o4,
                     &Simulator_B.SFunction_o2, temp_table, pres_table, 1);
    }

    /* Gain: '<S197>/reference area' incorporates:
     *  Gain: '<S83>/1//2rhoV^2'
     *  Product: '<S83>/Product2'
     *  Product: '<S85>/Product'
     *  Product: '<S85>/Product1'
     *  Product: '<S85>/Product2'
     *  Sum: '<S85>/Sum'
     */
    rtb_Fcn1 = ((rtb_Product_ly[0] * rtb_Product_ly[0] + rtb_Product_ly[1] *
                 rtb_Product_ly[1]) + rtb_Product_ly[2] * rtb_Product_ly[2]) *
      Simulator_B.SFunction_o4 * 0.5 * 0.535;

    /* UnitConversion: '<S300>/Unit Conversion' incorporates:
     *  Gain: '<S9>/xyH'
     *  Integrator: '<S60>/Integrator'
     */
    /* Unit Conversion - from: m to: ft
       Expression: output = (3.28084*input) + (0) */
    rtb_InterpolationUsingPrelookup = 3.280839895013123 *
      -Simulator_X.Integrator_CSTATE_a[2];

    /* Saturate: '<S255>/3ft-->inf' */
    if (rtb_InterpolationUsingPrelookup <= 3.0) {
      rtb_InterpolationUsingPrelookup = 3.0;
    }

    /* End of Saturate: '<S255>/3ft-->inf' */

    /* Gain: '<S255>/h//z0' */
    rtb_InterpolationUsingPrelookup *= 6.666666666666667;

    /* Math: '<S255>/ln(h//z0)'
     *
     * About '<S255>/ln(h//z0)':
     *  Operator: log
     */
    rtb_InterpolationUsingPrelookup = std::log(rtb_InterpolationUsingPrelookup);

    /* Product: '<S255>/Product' */
    rtb_Saturation_m = rtb_InterpolationUsingPrelookup / 4.8928522584398726;

    /* UnitConversion: '<S263>/Unit Conversion' incorporates:
     *  Gain: '<S9>/xyH'
     *  Integrator: '<S60>/Integrator'
     */
    /* Unit Conversion - from: m to: ft
       Expression: output = (3.28084*input) + (0) */
    rtb_UnitConversion = 3.280839895013123 * -Simulator_X.Integrator_CSTATE_a[2];

    /* DotProduct: '<S246>/Dot Product' */
    rtb_InterpolationUsingPrelookup = 0.0;
    for (i = 0; i < 3; i++) {
      /* Product: '<S255>/Transform from Inertial to Body axes' incorporates:
       *  Product: '<S255>/Product1'
       */
      rtb_LowAltitudeScaleLength = Simulator_B.VectorConcatenate[i + 6] *
        (rtb_Saturation_m * -0.0) + (Simulator_B.VectorConcatenate[i + 3] *
        (rtb_Saturation_m * -10.0) + rtb_Saturation_m * -6.1232339957367663E-16 *
        Simulator_B.VectorConcatenate[i]);

      /* Gain: '<S246>/Gain3' */
      rtb_Gain3[i] = 0.0 * rtb_LowAltitudeScaleLength;

      /* DotProduct: '<S246>/Dot Product' */
      rtb_InterpolationUsingPrelookup += rtb_Product_ly[i] * rtb_Product_ly[i];

      /* Product: '<S255>/Transform from Inertial to Body axes' */
      rtb_Integrator_o[i] = rtb_LowAltitudeScaleLength;
    }

    /* Sqrt: '<S246>/Sqrt' incorporates:
     *  DotProduct: '<S246>/Dot Product'
     */
    Simulator_B.Sqrt = std::sqrt(rtb_InterpolationUsingPrelookup);

    /* UnitConversion: '<S269>/Unit Conversion' */
    /* Unit Conversion - from: m/s to: ft/s
       Expression: output = (3.28084*input) + (0) */
    rtb_UnitConversion_i = 3.280839895013123 * Simulator_B.Sqrt;

    /* Saturate: '<S296>/Limit Function 10ft to 1000ft' incorporates:
     *  Saturate: '<S279>/Limit Height h<1000ft'
     */
    if (rtb_UnitConversion > 1000.0) {
      rtb_InterpolationUsingPrelookup = 1000.0;
      rtb_Saturation_m = 1000.0;
    } else {
      if (rtb_UnitConversion < 10.0) {
        rtb_InterpolationUsingPrelookup = 10.0;
      } else {
        rtb_InterpolationUsingPrelookup = rtb_UnitConversion;
      }

      if (rtb_UnitConversion < 0.0) {
        rtb_Saturation_m = 0.0;
      } else {
        rtb_Saturation_m = rtb_UnitConversion;
      }
    }

    /* End of Saturate: '<S296>/Limit Function 10ft to 1000ft' */

    /* Fcn: '<S296>/Low Altitude Scale Length' */
    rtb_LowAltitudeScaleLength = rtb_InterpolationUsingPrelookup / rt_powd_snf
      (0.000823 * rtb_InterpolationUsingPrelookup + 0.177, 1.2);

    /* Product: '<S279>/sigma_ug, sigma_vg' incorporates:
     *  Fcn: '<S279>/Low Altitude Intensity'
     */
    rtb_sigma_ugsigma_vg = 1.0 / rt_powd_snf(0.000823 * rtb_Saturation_m + 0.177,
      0.4) * 32.808398950131235;

    /* Interpolation_n-D: '<S278>/Medium//High Altitude Intensity' incorporates:
     *  PreLookup: '<S278>/PreLook-Up Index Search  (altitude)'
     */
    bpIndex[0] = plook_bincpa(rtb_UnitConversion,
      Simulator_ConstP.PreLookUpIndexSearchaltitude_Br, 11U, &rtb_Saturation_m,
      &Simulator_DW.PreLookUpIndexSearchaltitude_DW);
    frac[0] = rtb_Saturation_m;
    frac[1] = 0.0;
    bpIndex[1] = 2U;
    rtb_MediumHighAltitudeIntensity = intrp2d_la(bpIndex, frac,
      Simulator_ConstP.MediumHighAltitudeIntensity_Tab, 12U,
      Simulator_ConstP.MediumHighAltitudeIntensity_max);
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[2] == 0) {
      /* Product: '<S271>/Product' incorporates:
       *  RandomNumber: '<S271>/White Noise'
       */
      Simulator_B.Product[0] = 5.6049912163979281 * Simulator_DW.NextOutput[0];
      Simulator_B.Product[1] = 5.6049912163979281 * Simulator_DW.NextOutput[1];
      Simulator_B.Product[2] = 5.6049912163979281 * Simulator_DW.NextOutput[2];
      Simulator_B.Product[3] = 5.6049912163979281 * Simulator_DW.NextOutput[3];
    }

    /* Outputs for Enabled SubSystem: '<S262>/Hvgw(s)' incorporates:
     *  EnablePort: '<S276>/Enable'
     */
    /* Outputs for Enabled SubSystem: '<S262>/Hugw(s)' incorporates:
     *  EnablePort: '<S275>/Enable'
     */
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      if (rtmIsMajorTimeStep((&Simulator_M)) && Simulator_DW.Hugws_MODE) {
        /* Disable for Outport: '<S275>/ugw' */
        Simulator_B.w1_p[0] = 0.0;
        Simulator_B.w1_p[1] = 0.0;
        Simulator_DW.Hugws_MODE = false;
      }

      if (rtmIsMajorTimeStep((&Simulator_M)) && Simulator_DW.Hvgws_MODE) {
        /* Disable for Outport: '<S276>/vgw' */
        Simulator_B.w1[0] = 0.0;
        Simulator_B.w1[1] = 0.0;
        Simulator_DW.Hvgws_MODE = false;
      }
    }

    /* End of Outputs for SubSystem: '<S262>/Hvgw(s)' */
    if (Simulator_DW.Hugws_MODE) {
      /* Product: '<S275>/Lug//V' */
      rtb_Saturation_m = rtb_LowAltitudeScaleLength / rtb_UnitConversion_i;

      /* Product: '<S275>/w' incorporates:
       *  Gain: '<S275>/(2//pi)'
       *  Integrator: '<S275>/ug_p'
       *  Product: '<S275>/Lug//V'
       *  Product: '<S275>/Lug//V1'
       *  Sqrt: '<S275>/sqrt'
       *  Sum: '<S275>/Sum'
       */
      Simulator_B.w_e[0] = (std::sqrt(rtb_Saturation_m * 0.63661977236758138) *
                            Simulator_B.Product[0] - Simulator_X.ug_p_CSTATE[0])
        / rtb_Saturation_m;

      /* Product: '<S275>/Lug//V' */
      rtb_Saturation_m = Simulator_ConstB.UnitConversion_cm /
        rtb_UnitConversion_i;

      /* Product: '<S275>/w' incorporates:
       *  Gain: '<S275>/(2//pi)'
       *  Integrator: '<S275>/ug_p'
       *  Product: '<S275>/Lug//V'
       *  Product: '<S275>/Lug//V1'
       *  Sqrt: '<S275>/sqrt'
       *  Sum: '<S275>/Sum'
       */
      Simulator_B.w_e[1] = (std::sqrt(rtb_Saturation_m * 0.63661977236758138) *
                            Simulator_B.Product[0] - Simulator_X.ug_p_CSTATE[1])
        / rtb_Saturation_m;

      /* Product: '<S275>/w1' incorporates:
       *  Integrator: '<S275>/ug_p'
       */
      Simulator_B.w1_p[0] = Simulator_X.ug_p_CSTATE[0] * rtb_sigma_ugsigma_vg;
      Simulator_B.w1_p[1] = Simulator_X.ug_p_CSTATE[1] *
        rtb_MediumHighAltitudeIntensity;
    }

    /* End of Outputs for SubSystem: '<S262>/Hugw(s)' */

    /* Outputs for Enabled SubSystem: '<S262>/Hvgw(s)' incorporates:
     *  EnablePort: '<S276>/Enable'
     */
    if (Simulator_DW.Hvgws_MODE) {
      /* Product: '<S276>/Lvg//V' incorporates:
       *  Gain: '<S268>/Lv'
       */
      frac[0] = rtb_LowAltitudeScaleLength / rtb_UnitConversion_i;
      frac[1] = Simulator_ConstB.UnitConversion_cm / rtb_UnitConversion_i;

      /* Product: '<S276>/w' incorporates:
       *  Gain: '<S276>/(1//pi)'
       *  Integrator: '<S276>/vg_p1'
       *  Product: '<S276>/Lug//V1'
       *  Sqrt: '<S276>/sqrt'
       *  Sum: '<S276>/Sum'
       */
      Simulator_B.w_h[0] = (std::sqrt(0.31830988618379069 * frac[0]) *
                            Simulator_B.Product[1] - Simulator_X.vg_p1_CSTATE[0])
        / frac[0];

      /* Product: '<S276>/w ' incorporates:
       *  Gain: '<S276>/(1//pi)'
       *  Gain: '<S276>/sqrt(3)'
       *  Integrator: '<S276>/vg_p1'
       *  Integrator: '<S276>/vgw_p2'
       *  Product: '<S276>/Lvg//V '
       *  Sum: '<S276>/Sum1'
       */
      Simulator_B.w_j[0] = (Simulator_B.w_h[0] * frac[0] * 1.7320508075688772 +
                            (Simulator_X.vg_p1_CSTATE[0] -
        Simulator_X.vgw_p2_CSTATE[0])) / frac[0];

      /* Product: '<S276>/w' incorporates:
       *  Gain: '<S276>/(1//pi)'
       *  Integrator: '<S276>/vg_p1'
       *  Product: '<S276>/Lug//V1'
       *  Sqrt: '<S276>/sqrt'
       *  Sum: '<S276>/Sum'
       */
      Simulator_B.w_h[1] = (std::sqrt(0.31830988618379069 * frac[1]) *
                            Simulator_B.Product[1] - Simulator_X.vg_p1_CSTATE[1])
        / frac[1];

      /* Product: '<S276>/w ' incorporates:
       *  Gain: '<S276>/(1//pi)'
       *  Gain: '<S276>/sqrt(3)'
       *  Integrator: '<S276>/vg_p1'
       *  Integrator: '<S276>/vgw_p2'
       *  Product: '<S276>/Lvg//V '
       *  Sum: '<S276>/Sum1'
       */
      Simulator_B.w_j[1] = (Simulator_B.w_h[1] * frac[1] * 1.7320508075688772 +
                            (Simulator_X.vg_p1_CSTATE[1] -
        Simulator_X.vgw_p2_CSTATE[1])) / frac[1];

      /* Product: '<S276>/w 1' incorporates:
       *  Integrator: '<S276>/vgw_p2'
       */
      Simulator_B.w1[0] = rtb_sigma_ugsigma_vg * Simulator_X.vgw_p2_CSTATE[0];
      Simulator_B.w1[1] = rtb_MediumHighAltitudeIntensity *
        Simulator_X.vgw_p2_CSTATE[1];
    }

    /* End of Outputs for SubSystem: '<S262>/Hvgw(s)' */

    /* Gain: '<S268>/Lw' */
    frac[0] = rtb_InterpolationUsingPrelookup;

    /* Outputs for Enabled SubSystem: '<S262>/Hwgw(s)' incorporates:
     *  EnablePort: '<S277>/Enable'
     */
    if ((rtmIsMajorTimeStep((&Simulator_M)) &&
         (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) && rtmIsMajorTimeStep
        ((&Simulator_M)) && Simulator_DW.Hwgws_MODE) {
      /* Disable for Outport: '<S277>/wgw' */
      Simulator_B.LwgV1[0] = 0.0;
      Simulator_B.LwgV1[1] = 0.0;
      Simulator_DW.Hwgws_MODE = false;
    }

    if (Simulator_DW.Hwgws_MODE) {
      /* Product: '<S277>/Lwg//V 1' incorporates:
       *  Integrator: '<S277>/wg_p2'
       */
      Simulator_B.LwgV1[0] = 32.808398950131235 * Simulator_X.wg_p2_CSTATE[0];
      Simulator_B.LwgV1[1] = rtb_MediumHighAltitudeIntensity *
        Simulator_X.wg_p2_CSTATE[1];

      /* Product: '<S277>/Lwg//V' incorporates:
       *  Gain: '<S268>/Lw'
       */
      rtb_Saturation_m = rtb_InterpolationUsingPrelookup / rtb_UnitConversion_i;

      /* Product: '<S277>/w' incorporates:
       *  Gain: '<S277>/1//pi'
       *  Integrator: '<S277>/wg_p1'
       *  Product: '<S277>/Lug//V1'
       *  Sqrt: '<S277>/sqrt1'
       *  Sum: '<S277>/Sum'
       */
      Simulator_B.w[0] = (std::sqrt(0.31830988618379069 * rtb_Saturation_m) *
                          Simulator_B.Product[2] - Simulator_X.wg_p1_CSTATE[0]) /
        rtb_Saturation_m;

      /* Product: '<S277>/w ' incorporates:
       *  Integrator: '<S277>/wg_p1'
       *  Integrator: '<S277>/wg_p2'
       *  Product: '<S277>/Lwg//V '
       *  Sum: '<S277>/Sum1'
       */
      Simulator_B.w_k[0] = (Simulator_B.w[0] * 1.7320508075688772 *
                            rtb_Saturation_m + (Simulator_X.wg_p1_CSTATE[0] -
        Simulator_X.wg_p2_CSTATE[0])) / rtb_Saturation_m;

      /* Product: '<S277>/Lwg//V' incorporates:
       *  Gain: '<S268>/Lw'
       */
      rtb_Saturation_m = Simulator_ConstB.UnitConversion_cm /
        rtb_UnitConversion_i;

      /* Product: '<S277>/w' incorporates:
       *  Gain: '<S277>/1//pi'
       *  Integrator: '<S277>/wg_p1'
       *  Product: '<S277>/Lug//V1'
       *  Sqrt: '<S277>/sqrt1'
       *  Sum: '<S277>/Sum'
       */
      Simulator_B.w[1] = (std::sqrt(0.31830988618379069 * rtb_Saturation_m) *
                          Simulator_B.Product[2] - Simulator_X.wg_p1_CSTATE[1]) /
        rtb_Saturation_m;

      /* Product: '<S277>/w ' incorporates:
       *  Integrator: '<S277>/wg_p1'
       *  Integrator: '<S277>/wg_p2'
       *  Product: '<S277>/Lwg//V '
       *  Sum: '<S277>/Sum1'
       */
      Simulator_B.w_k[1] = (Simulator_B.w[1] * 1.7320508075688772 *
                            rtb_Saturation_m + (Simulator_X.wg_p1_CSTATE[1] -
        Simulator_X.wg_p2_CSTATE[1])) / rtb_Saturation_m;
    }

    /* End of Outputs for SubSystem: '<S262>/Hwgw(s)' */

    /* If: '<S267>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
    if (rtmIsMajorTimeStep((&Simulator_M))) {
      if (rtb_UnitConversion <= 1000.0) {
        rtAction = 0;
      } else if (rtb_UnitConversion >= 2000.0) {
        rtAction = 1;
      } else {
        rtAction = 2;
      }

      Simulator_DW.ifHeightMaxlowaltitudeelseifHei = rtAction;
    } else {
      rtAction = Simulator_DW.ifHeightMaxlowaltitudeelseifHei;
    }

    switch (rtAction) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S267>/Low altitude  velocities' incorporates:
       *  ActionPort: '<S289>/Action Port'
       */
      /* Sum: '<S295>/Sum' incorporates:
       *  Product: '<S295>/Product1'
       *  Product: '<S295>/Product2'
       */
      rtb_Sum2_m[0] = Simulator_B.w1_p[0] * 6.123233995736766E-17 -
        Simulator_B.w1[0];

      /* Sum: '<S295>/Sum1' incorporates:
       *  Product: '<S295>/Product1'
       *  Product: '<S295>/Product2'
       */
      rtb_Sum2_m[1] = Simulator_B.w1[0] * 6.123233995736766E-17 +
        Simulator_B.w1_p[0];

      /* Reshape: '<S294>/Reshape1' incorporates:
       *  Product: '<S294>/Product'
       *  SignalConversion generated from: '<S294>/Vector Concatenate'
       */
      for (i = 0; i < 3; i++) {
        rtb_Integrator_o[i] = Simulator_B.VectorConcatenate[i + 6] *
          Simulator_B.LwgV1[0] + (Simulator_B.VectorConcatenate[i + 3] *
          rtb_Sum2_m[1] + Simulator_B.VectorConcatenate[i] * rtb_Sum2_m[0]);
      }

      /* End of Reshape: '<S294>/Reshape1' */
      /* End of Outputs for SubSystem: '<S267>/Low altitude  velocities' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S267>/Medium//High  altitude velocities' incorporates:
       *  ActionPort: '<S290>/Action Port'
       */
      /* Gain: '<S290>/Gain' */
      rtb_Integrator_o[0] = Simulator_B.w1_p[1];
      rtb_Integrator_o[1] = Simulator_B.w1[1];
      rtb_Integrator_o[2] = Simulator_B.LwgV1[1];

      /* End of Outputs for SubSystem: '<S267>/Medium//High  altitude velocities' */
      break;

     case 2:
      /* Outputs for IfAction SubSystem: '<S267>/Interpolate  velocities' incorporates:
       *  ActionPort: '<S288>/Action Port'
       */
      /* Sum: '<S293>/Sum' incorporates:
       *  Product: '<S293>/Product1'
       *  Product: '<S293>/Product2'
       */
      rtb_Integrator_o[0] = Simulator_B.w1_p[0] * 6.123233995736766E-17 -
        Simulator_B.w1[0];

      /* Sum: '<S293>/Sum1' incorporates:
       *  Product: '<S293>/Product1'
       *  Product: '<S293>/Product2'
       */
      rtb_Integrator_o[1] = Simulator_B.w1[0] * 6.123233995736766E-17 +
        Simulator_B.w1_p[0];

      /* Product: '<S292>/Product' incorporates:
       *  SignalConversion generated from: '<S292>/Vector Concatenate'
       */
      for (i = 0; i < 3; i++) {
        rtb_Sum2_m[i] = Simulator_B.VectorConcatenate[i + 6] *
          Simulator_B.LwgV1[0] + (Simulator_B.VectorConcatenate[i + 3] *
          rtb_Integrator_o[1] + Simulator_B.VectorConcatenate[i] *
          rtb_Integrator_o[0]);
      }

      /* End of Product: '<S292>/Product' */

      /* Sum: '<S288>/Sum3' incorporates:
       *  Constant: '<S288>/max_height_low'
       *  Product: '<S288>/Product1'
       *  Sum: '<S288>/Sum1'
       *  Sum: '<S288>/Sum2'
       */
      rtb_Integrator_o[0] = (Simulator_B.w1_p[1] - rtb_Sum2_m[0]) *
        (rtb_UnitConversion - 1000.0) / 1000.0 + rtb_Sum2_m[0];
      rtb_Integrator_o[1] = (Simulator_B.w1[1] - rtb_Sum2_m[1]) *
        (rtb_UnitConversion - 1000.0) / 1000.0 + rtb_Sum2_m[1];
      rtb_Integrator_o[2] = (Simulator_B.LwgV1[1] - rtb_Sum2_m[2]) *
        (rtb_UnitConversion - 1000.0) / 1000.0 + rtb_Sum2_m[2];

      /* End of Outputs for SubSystem: '<S267>/Interpolate  velocities' */
      break;
    }

    /* End of If: '<S267>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */

    /* UnitConversion: '<S254>/Unit Conversion' */
    /* Unit Conversion - from: ft/s to: m/s
       Expression: output = (0.3048*input) + (0) */
    rtb_Integrator_o[0] *= 0.3048;
    rtb_Integrator_o[1] *= 0.3048;

    /* Logic: '<S253>/Logical Operator2' incorporates:
     *  Constant: '<S253>/Constant'
     */
    Simulator_B.LogicalOperator2 = false;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* SignalConversion generated from: '<S256>/Enable' */
      Simulator_B.HiddenBuf_InsertedFor_Distancei = Simulator_B.LogicalOperator2;

      /* Outputs for Enabled SubSystem: '<S253>/Distance into gust (x)' incorporates:
       *  EnablePort: '<S256>/Enable'
       */
      if (rtmIsMajorTimeStep((&Simulator_M))) {
        if (Simulator_B.HiddenBuf_InsertedFor_Distancei) {
          if (!Simulator_DW.Distanceintogustx_MODE) {
            /* InitializeConditions for Integrator: '<S256>/Distance into Gust (x) (Limited to gust length d)' */
            Simulator_X.DistanceintoGustxLimitedtogus_n = 0.0;
            Simulator_DW.Distanceintogustx_MODE = true;
          }
        } else {
          Simulator_DW.Distanceintogustx_MODE = false;
        }
      }

      /* End of Outputs for SubSystem: '<S253>/Distance into gust (x)' */
    }

    /* Outputs for Enabled SubSystem: '<S253>/Distance into gust (x)' incorporates:
     *  EnablePort: '<S256>/Enable'
     */
    if (Simulator_DW.Distanceintogustx_MODE) {
      /* Integrator: '<S256>/Distance into Gust (x) (Limited to gust length d)' */
      /* Limited  Integrator  */
      if (Simulator_X.DistanceintoGustxLimitedtogus_n >= 120.0) {
        Simulator_X.DistanceintoGustxLimitedtogus_n = 120.0;
      } else {
        if (Simulator_X.DistanceintoGustxLimitedtogus_n <= 0.0) {
          Simulator_X.DistanceintoGustxLimitedtogus_n = 0.0;
        }
      }

      Simulator_B.DistanceintoGustxLimitedtogustl =
        Simulator_X.DistanceintoGustxLimitedtogus_n;

      /* End of Integrator: '<S256>/Distance into Gust (x) (Limited to gust length d)' */
    }

    /* End of Outputs for SubSystem: '<S253>/Distance into gust (x)' */

    /* Logic: '<S253>/Logical Operator1' incorporates:
     *  Constant: '<S253>/Constant1'
     */
    Simulator_B.LogicalOperator1_o = false;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* SignalConversion generated from: '<S257>/Enable' */
      Simulator_B.HiddenBuf_InsertedFor_Distanc_m =
        Simulator_B.LogicalOperator1_o;
    }

    /* Outputs for Enabled SubSystem: '<S253>/Distance into gust (y)' */
    Simulator_Distanceintogusty(Simulator_B.HiddenBuf_InsertedFor_Distanc_m,
      &Simulator_B.Distanceintogusty, &Simulator_DW.Distanceintogusty,
      &Simulator_X.Distanceintogusty, 120.0);

    /* End of Outputs for SubSystem: '<S253>/Distance into gust (y)' */

    /* Logic: '<S253>/Logical Operator3' incorporates:
     *  Constant: '<S253>/Constant2'
     */
    Simulator_B.LogicalOperator3 = false;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* SignalConversion generated from: '<S258>/Enable' */
      Simulator_B.HiddenBuf_InsertedFor_Distanc_l = Simulator_B.LogicalOperator3;
    }

    /* Outputs for Enabled SubSystem: '<S253>/Distance into gust (z)' */
    Simulator_Distanceintogusty(Simulator_B.HiddenBuf_InsertedFor_Distanc_l,
      &Simulator_B.Distanceintogustz, &Simulator_DW.Distanceintogustz,
      &Simulator_X.Distanceintogustz, 80.0);

    /* End of Outputs for SubSystem: '<S253>/Distance into gust (z)' */

    /* Sum: '<S69>/Sum4' incorporates:
     *  Constant: '<S253>/2'
     *  Gain: '<S246>/Gain'
     *  Gain: '<S246>/Gain1'
     *  Gain: '<S253>/Gust magnitude//2.0'
     *  Gain: '<S253>/pi//Gust length'
     *  Sum: '<S246>/Sum'
     *  Sum: '<S253>/Sum1'
     *  Trigonometry: '<S253>/cos(pi*x//dm)'
     *  UnitConversion: '<S254>/Unit Conversion'
     */
    rtb_Integrator_o[0] = rtb_Mstabtobody[0] - ((1.0 - std::cos
      (0.026179938779914941 * Simulator_B.DistanceintoGustxLimitedtogustl)) *
      1.5 * 0.0 + (0.0 * rtb_Integrator_o[0] + rtb_Gain3[0]));
    rtb_Integrator_o[1] = rtb_Mstabtobody[1] - ((1.0 - std::cos
      (0.026179938779914941 *
       Simulator_B.Distanceintogusty.DistanceintoGustxLimitedtogustl)) * 1.5 *
      0.0 + (0.0 * rtb_Integrator_o[1] + rtb_Gain3[1]));
    rtb_LowAltitudeScaleLength = rtb_Mstabtobody[2] - ((1.0 - std::cos
      (0.039269908169872414 *
       Simulator_B.Distanceintogustz.DistanceintoGustxLimitedtogustl)) * 1.5 *
      0.0 + (0.3048 * rtb_Integrator_o[2] * 0.0 + rtb_Gain3[2]));
    rtb_Integrator_o[2] = rtb_LowAltitudeScaleLength;

    /* Gain: '<S185>/Gain' incorporates:
     *  Trigonometry: '<S84>/Incidence'
     */
    rtb_InterpolationUsingPrelookup = 57.295779513082323 * rt_atan2d_snf
      (rtb_LowAltitudeScaleLength, rtb_Integrator_o[0]);

    /* PreLookup generated from: '<S92>/Prelookup' */
    rtb_k_b = plook_binc(rtb_InterpolationUsingPrelookup,
                         Simulator_ConstP.pooled28, 17U,
                         &rtb_InterpolationUsingPrelookup);

    /* Gain: '<S184>/Gain1' incorporates:
     *  Constant: '<S92>/Constant'
     *  Interpolation_n-D generated from: '<S92>/Interpolation Using Prelookup'
     */
    rtb_Saturation_m = 0.017453292519943295 * intrp1d_l(rtb_k_b,
      rtb_InterpolationUsingPrelookup, Simulator_ConstP.pooled28);
    rtb_Product2_n_idx_1 = 0.0;

    /* Trigonometry: '<S183>/sincos' */
    rtb_pgw_p_idx_0 = std::cos(rtb_Saturation_m);
    rtb_Saturation_m = std::sin(rtb_Saturation_m);

    /* Product: '<S187>/u(3)*u(4)' */
    rtb_VectorConcatenate[0] = rtb_pgw_p_idx_0;

    /* UnaryMinus: '<S190>/Unary Minus' incorporates:
     *  Product: '<S190>/u(2)*u(3)'
     */
    rtb_VectorConcatenate[1] = -(0.0 * rtb_pgw_p_idx_0);

    /* UnaryMinus: '<S193>/Unary Minus' */
    rtb_VectorConcatenate[2] = -rtb_Saturation_m;

    /* SignalConversion generated from: '<S196>/Vector Concatenate' */
    rtb_VectorConcatenate[3] = 0.0;

    /* SignalConversion generated from: '<S196>/Vector Concatenate' */
    rtb_VectorConcatenate[4] = 1.0;

    /* SignalConversion generated from: '<S196>/Vector Concatenate' incorporates:
     *  Constant: '<S194>/Constant'
     */
    rtb_VectorConcatenate[5] = 0.0;

    /* Product: '<S189>/u(1)*u(4)' */
    rtb_VectorConcatenate[6] = rtb_Saturation_m;

    /* UnaryMinus: '<S192>/Unary Minus' incorporates:
     *  Product: '<S192>/u(1)*u(2)'
     */
    rtb_VectorConcatenate[7] = -(rtb_Saturation_m * 0.0);

    /* SignalConversion generated from: '<S196>/Vector Concatenate' */
    rtb_VectorConcatenate[8] = rtb_pgw_p_idx_0;

    /* Math: '<S92>/Stab to body' */
    for (i = 0; i < 3; i++) {
      rtb_DCM_bs[3 * i] = rtb_VectorConcatenate[i];
      rtb_DCM_bs[3 * i + 1] = rtb_VectorConcatenate[i + 3];
      rtb_DCM_bs[3 * i + 2] = rtb_VectorConcatenate[i + 6];
    }

    /* End of Math: '<S92>/Stab to body' */

    /* Sqrt: '<S84>/Airspeed' incorporates:
     *  Product: '<S88>/Product'
     *  Product: '<S88>/Product1'
     *  Product: '<S88>/Product2'
     *  Sum: '<S88>/Sum'
     */
    rtb_Saturation_m = std::sqrt((rtb_Integrator_o[0] * rtb_Integrator_o[0] +
      rtb_Integrator_o[1] * rtb_Integrator_o[1]) + rtb_LowAltitudeScaleLength *
      rtb_LowAltitudeScaleLength);

    /* Saturate: '<S92>/Saturation' */
    if (rtb_Saturation_m > 100.0) {
      rtb_LowAltitudeScaleLength = 100.0;
    } else if (rtb_Saturation_m < 0.0) {
      rtb_LowAltitudeScaleLength = 0.0;
    } else {
      rtb_LowAltitudeScaleLength = rtb_Saturation_m;
    }

    /* End of Saturate: '<S92>/Saturation' */

    /* PreLookup generated from: '<S92>/Prelookup2' */
    rtb_k = plook_binx(rtb_LowAltitudeScaleLength,
                       Simulator_ConstP.Prelookup2_1_BreakpointsData, 11U,
                       &rtb_LowAltitudeScaleLength);

    /* Interpolation_n-D generated from: '<S154>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S154>/Constant'
     *  Constant: '<S154>/Constant3'
     */
    frac_0[0] = rtb_InterpolationUsingPrelookup;
    frac_0[1] = 0.0;
    frac_0[2] = rtb_LowAltitudeScaleLength;
    bpIndex_0[0] = rtb_k_b;
    bpIndex_0[1] = 0U;
    bpIndex_0[2] = rtb_k;
    rtb_sigma_ugsigma_vg = intrp3d_l(bpIndex_0, frac_0,
      Simulator_ConstP.pooled30, Simulator_ConstP.pooled91);

    /* Saturate: '<S84>/Saturation' */
    if (rtb_Saturation_m <= 0.1) {
      rtb_thetadot = 0.1;
    } else {
      rtb_thetadot = rtb_Saturation_m;
    }

    /* End of Saturate: '<S84>/Saturation' */

    /* Product: '<S84>/Product' */
    rtb_thetadot = rtb_Integrator_o[1] / rtb_thetadot;

    /* Trigonometry: '<S84>/Sideslip' */
    if (rtb_thetadot > 1.0) {
      rtb_thetadot = 1.0;
    } else {
      if (rtb_thetadot < -1.0) {
        rtb_thetadot = -1.0;
      }
    }

    rtb_Sideslip = std::asin(rtb_thetadot);

    /* End of Trigonometry: '<S84>/Sideslip' */

    /* Gain: '<S186>/Gain' incorporates:
     *  Abs: '<S92>/Abs'
     */
    rtb_f_n = 57.295779513082323 * std::abs(rtb_Sideslip);

    /* PreLookup generated from: '<S92>/Prelookup1' */
    rtb_k_d = plook_binc(rtb_f_n, Simulator_ConstP.Prelookup1_1_BreakpointsData,
                         10U, &rtb_f_n);

    /* Interpolation_n-D generated from: '<S154>/Interpolation Using Prelookup' */
    frac_1[0] = rtb_InterpolationUsingPrelookup;
    frac_1[1] = rtb_f_n;
    frac_1[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1[0] = rtb_k_b;
    bpIndex_1[1] = rtb_k_d;
    bpIndex_1[2] = rtb_k;
    rtb_Add_bm = intrp3d_l(bpIndex_1, frac_1, Simulator_ConstP.pooled30,
      Simulator_ConstP.pooled91);

    /* Sum: '<S154>/Add' incorporates:
     *  Sum: '<S154>/Subtract'
     */
    rtb_sigma_ugsigma_vg += rtb_Add_bm - rtb_sigma_ugsigma_vg;

    /* Interpolation_n-D generated from: '<S155>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S155>/Constant'
     *  Constant: '<S155>/Constant3'
     */
    frac_2[0] = rtb_InterpolationUsingPrelookup;
    frac_2[1] = 0.0;
    frac_2[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2[0] = rtb_k_b;
    bpIndex_2[1] = 0U;
    bpIndex_2[2] = rtb_k;
    rtb_Add_bm = intrp3d_l(bpIndex_2, frac_2, Simulator_ConstP.pooled32,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S155>/Interpolation Using Prelookup' */
    frac_3[0] = rtb_InterpolationUsingPrelookup;
    frac_3[1] = rtb_f_n;
    frac_3[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3[0] = rtb_k_b;
    bpIndex_3[1] = rtb_k_d;
    bpIndex_3[2] = rtb_k;
    rtb_Add_kh = intrp3d_l(bpIndex_3, frac_3, Simulator_ConstP.pooled32,
      Simulator_ConstP.pooled91);

    /* Signum: '<S155>/Sign' */
    if (rtb_Sideslip < 0.0) {
      /* Signum: '<S156>/Sign' incorporates:
       *  Signum: '<S102>/Sign'
       *  Signum: '<S103>/Sign'
       *  Signum: '<S106>/Sign'
       *  Signum: '<S111>/Sign'
       *  Signum: '<S112>/Sign'
       *  Signum: '<S114>/Sign'
       *  Signum: '<S118>/Sign'
       *  Signum: '<S119>/Sign'
       *  Signum: '<S121>/Sign'
       *  Signum: '<S123>/Sign'
       *  Signum: '<S124>/Sign'
       *  Signum: '<S127>/Sign'
       *  Signum: '<S130>/Sign'
       *  Signum: '<S131>/Sign'
       *  Signum: '<S134>/Sign'
       *  Signum: '<S141>/Sign'
       *  Signum: '<S142>/Sign'
       *  Signum: '<S144>/Sign'
       *  Signum: '<S146>/Sign'
       *  Signum: '<S147>/Sign'
       *  Signum: '<S150>/Sign'
       *  Signum: '<S158>/Sign'
       *  Signum: '<S163>/Sign'
       *  Signum: '<S164>/Sign'
       *  Signum: '<S167>/Sign'
       *  Signum: '<S172>/Sign'
       *  Signum: '<S173>/Sign'
       *  Signum: '<S175>/Sign'
       *  Signum: '<S177>/Sign'
       *  Signum: '<S178>/Sign'
       *  Signum: '<S181>/Sign'
       */
      rtb_Sideslip = -1.0;
    } else if (rtb_Sideslip > 0.0) {
      /* Signum: '<S156>/Sign' incorporates:
       *  Signum: '<S102>/Sign'
       *  Signum: '<S103>/Sign'
       *  Signum: '<S106>/Sign'
       *  Signum: '<S111>/Sign'
       *  Signum: '<S112>/Sign'
       *  Signum: '<S114>/Sign'
       *  Signum: '<S118>/Sign'
       *  Signum: '<S119>/Sign'
       *  Signum: '<S121>/Sign'
       *  Signum: '<S123>/Sign'
       *  Signum: '<S124>/Sign'
       *  Signum: '<S127>/Sign'
       *  Signum: '<S130>/Sign'
       *  Signum: '<S131>/Sign'
       *  Signum: '<S134>/Sign'
       *  Signum: '<S141>/Sign'
       *  Signum: '<S142>/Sign'
       *  Signum: '<S144>/Sign'
       *  Signum: '<S146>/Sign'
       *  Signum: '<S147>/Sign'
       *  Signum: '<S150>/Sign'
       *  Signum: '<S158>/Sign'
       *  Signum: '<S163>/Sign'
       *  Signum: '<S164>/Sign'
       *  Signum: '<S167>/Sign'
       *  Signum: '<S172>/Sign'
       *  Signum: '<S173>/Sign'
       *  Signum: '<S175>/Sign'
       *  Signum: '<S177>/Sign'
       *  Signum: '<S178>/Sign'
       *  Signum: '<S181>/Sign'
       */
      rtb_Sideslip = 1.0;
    } else if (rtb_Sideslip == 0.0) {
      /* Signum: '<S156>/Sign' incorporates:
       *  Signum: '<S102>/Sign'
       *  Signum: '<S103>/Sign'
       *  Signum: '<S106>/Sign'
       *  Signum: '<S111>/Sign'
       *  Signum: '<S112>/Sign'
       *  Signum: '<S114>/Sign'
       *  Signum: '<S118>/Sign'
       *  Signum: '<S119>/Sign'
       *  Signum: '<S121>/Sign'
       *  Signum: '<S123>/Sign'
       *  Signum: '<S124>/Sign'
       *  Signum: '<S127>/Sign'
       *  Signum: '<S130>/Sign'
       *  Signum: '<S131>/Sign'
       *  Signum: '<S134>/Sign'
       *  Signum: '<S141>/Sign'
       *  Signum: '<S142>/Sign'
       *  Signum: '<S144>/Sign'
       *  Signum: '<S146>/Sign'
       *  Signum: '<S147>/Sign'
       *  Signum: '<S150>/Sign'
       *  Signum: '<S158>/Sign'
       *  Signum: '<S163>/Sign'
       *  Signum: '<S164>/Sign'
       *  Signum: '<S167>/Sign'
       *  Signum: '<S172>/Sign'
       *  Signum: '<S173>/Sign'
       *  Signum: '<S175>/Sign'
       *  Signum: '<S177>/Sign'
       *  Signum: '<S178>/Sign'
       *  Signum: '<S181>/Sign'
       */
      rtb_Sideslip = 0.0;
    } else {
      /* Signum: '<S156>/Sign' incorporates:
       *  Signum: '<S102>/Sign'
       *  Signum: '<S103>/Sign'
       *  Signum: '<S106>/Sign'
       *  Signum: '<S111>/Sign'
       *  Signum: '<S112>/Sign'
       *  Signum: '<S114>/Sign'
       *  Signum: '<S118>/Sign'
       *  Signum: '<S119>/Sign'
       *  Signum: '<S121>/Sign'
       *  Signum: '<S123>/Sign'
       *  Signum: '<S124>/Sign'
       *  Signum: '<S127>/Sign'
       *  Signum: '<S130>/Sign'
       *  Signum: '<S131>/Sign'
       *  Signum: '<S134>/Sign'
       *  Signum: '<S141>/Sign'
       *  Signum: '<S142>/Sign'
       *  Signum: '<S144>/Sign'
       *  Signum: '<S146>/Sign'
       *  Signum: '<S147>/Sign'
       *  Signum: '<S150>/Sign'
       *  Signum: '<S158>/Sign'
       *  Signum: '<S163>/Sign'
       *  Signum: '<S164>/Sign'
       *  Signum: '<S167>/Sign'
       *  Signum: '<S172>/Sign'
       *  Signum: '<S173>/Sign'
       *  Signum: '<S175>/Sign'
       *  Signum: '<S177>/Sign'
       *  Signum: '<S178>/Sign'
       *  Signum: '<S181>/Sign'
       */
      rtb_Sideslip = (rtNaN);
    }

    /* Sum: '<S155>/Add' incorporates:
     *  Product: '<S155>/Product'
     *  Signum: '<S155>/Sign'
     *  Sum: '<S155>/Subtract'
     */
    rtb_Add_bm += (rtb_Add_kh - rtb_Add_bm) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S153>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S153>/Constant'
     *  Constant: '<S153>/Constant3'
     */
    frac_4[0] = rtb_InterpolationUsingPrelookup;
    frac_4[1] = 0.0;
    frac_4[2] = rtb_LowAltitudeScaleLength;
    bpIndex_4[0] = rtb_k_b;
    bpIndex_4[1] = 0U;
    bpIndex_4[2] = rtb_k;
    rtb_Add_kh = intrp3d_l(bpIndex_4, frac_4, Simulator_ConstP.pooled33,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S153>/Interpolation Using Prelookup' */
    frac_5[0] = rtb_InterpolationUsingPrelookup;
    frac_5[1] = rtb_f_n;
    frac_5[2] = rtb_LowAltitudeScaleLength;
    bpIndex_5[0] = rtb_k_b;
    bpIndex_5[1] = rtb_k_d;
    bpIndex_5[2] = rtb_k;
    rtb_CLtoCz = intrp3d_l(bpIndex_5, frac_5, Simulator_ConstP.pooled33,
      Simulator_ConstP.pooled91);

    /* Gain: '<S152>/CL to Cz' incorporates:
     *  Sum: '<S153>/Add'
     *  Sum: '<S153>/Subtract'
     */
    rtb_CLtoCz = -((rtb_CLtoCz - rtb_Add_kh) + rtb_Add_kh);

    /* Interpolation_n-D generated from: '<S156>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S156>/Constant'
     *  Constant: '<S156>/Constant3'
     */
    frac_6[0] = rtb_InterpolationUsingPrelookup;
    frac_6[1] = 0.0;
    frac_6[2] = rtb_LowAltitudeScaleLength;
    bpIndex_6[0] = rtb_k_b;
    bpIndex_6[1] = 0U;
    bpIndex_6[2] = rtb_k;
    rtb_Add_kh = intrp3d_l(bpIndex_6, frac_6, Simulator_ConstP.pooled34,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S156>/Interpolation Using Prelookup' */
    frac_7[0] = rtb_InterpolationUsingPrelookup;
    frac_7[1] = rtb_f_n;
    frac_7[2] = rtb_LowAltitudeScaleLength;
    bpIndex_7[0] = rtb_k_b;
    bpIndex_7[1] = rtb_k_d;
    bpIndex_7[2] = rtb_k;
    rtb_Add_bc = intrp3d_l(bpIndex_7, frac_7, Simulator_ConstP.pooled34,
      Simulator_ConstP.pooled91);

    /* Sum: '<S156>/Add' incorporates:
     *  Product: '<S156>/Product'
     *  Sum: '<S156>/Subtract'
     */
    rtb_Add_kh += (rtb_Add_bc - rtb_Add_kh) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S157>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S157>/Constant'
     *  Constant: '<S157>/Constant3'
     */
    frac_8[0] = rtb_InterpolationUsingPrelookup;
    frac_8[1] = 0.0;
    frac_8[2] = rtb_LowAltitudeScaleLength;
    bpIndex_8[0] = rtb_k_b;
    bpIndex_8[1] = 0U;
    bpIndex_8[2] = rtb_k;
    rtb_Add_bc = intrp3d_l(bpIndex_8, frac_8, Simulator_ConstP.pooled35,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S157>/Interpolation Using Prelookup' */
    frac_9[0] = rtb_InterpolationUsingPrelookup;
    frac_9[1] = rtb_f_n;
    frac_9[2] = rtb_LowAltitudeScaleLength;
    bpIndex_9[0] = rtb_k_b;
    bpIndex_9[1] = rtb_k_d;
    bpIndex_9[2] = rtb_k;
    rtb_Add_hq = intrp3d_l(bpIndex_9, frac_9, Simulator_ConstP.pooled35,
      Simulator_ConstP.pooled91);

    /* Sum: '<S157>/Add' incorporates:
     *  Sum: '<S157>/Subtract'
     */
    rtb_Add_bc += rtb_Add_hq - rtb_Add_bc;

    /* Interpolation_n-D generated from: '<S158>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S158>/Constant'
     *  Constant: '<S158>/Constant3'
     */
    frac_a[0] = rtb_InterpolationUsingPrelookup;
    frac_a[1] = 0.0;
    frac_a[2] = rtb_LowAltitudeScaleLength;
    bpIndex_a[0] = rtb_k_b;
    bpIndex_a[1] = 0U;
    bpIndex_a[2] = rtb_k;
    rtb_Add_hq = intrp3d_l(bpIndex_a, frac_a, Simulator_ConstP.pooled36,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S158>/Interpolation Using Prelookup' */
    frac_b[0] = rtb_InterpolationUsingPrelookup;
    frac_b[1] = rtb_f_n;
    frac_b[2] = rtb_LowAltitudeScaleLength;
    bpIndex_b[0] = rtb_k_b;
    bpIndex_b[1] = rtb_k_d;
    bpIndex_b[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_b, frac_b, Simulator_ConstP.pooled36,
      Simulator_ConstP.pooled91);

    /* Sum: '<S158>/Add' incorporates:
     *  Product: '<S158>/Product'
     *  Sum: '<S158>/Subtract'
     */
    rtb_Add_hq += (rtb_pgw_p_idx_0 - rtb_Add_hq) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S103>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S103>/Constant'
     *  Constant: '<S103>/Constant3'
     */
    frac_c[0] = rtb_InterpolationUsingPrelookup;
    frac_c[1] = 0.0;
    frac_c[2] = rtb_LowAltitudeScaleLength;
    bpIndex_c[0] = rtb_k_b;
    bpIndex_c[1] = 0U;
    bpIndex_c[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_c, frac_c, Simulator_ConstP.pooled37,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S103>/Interpolation Using Prelookup' */
    frac_d[0] = rtb_InterpolationUsingPrelookup;
    frac_d[1] = rtb_f_n;
    frac_d[2] = rtb_LowAltitudeScaleLength;
    bpIndex_d[0] = rtb_k_b;
    bpIndex_d[1] = rtb_k_d;
    bpIndex_d[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_d, frac_d, Simulator_ConstP.pooled37,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S103>/Add' incorporates:
     *  Product: '<S103>/Product'
     *  Sum: '<S103>/Subtract'
     */
    rtb_pgw_p_idx_0 += (rtb_jxi - rtb_pgw_p_idx_0) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S104>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S104>/Constant'
     *  Constant: '<S104>/Constant3'
     */
    frac_e[0] = rtb_InterpolationUsingPrelookup;
    frac_e[1] = 0.0;
    frac_e[2] = rtb_LowAltitudeScaleLength;
    bpIndex_e[0] = rtb_k_b;
    bpIndex_e[1] = 0U;
    bpIndex_e[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_e, frac_e, Simulator_ConstP.pooled38,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S104>/Interpolation Using Prelookup' */
    frac_f[0] = rtb_InterpolationUsingPrelookup;
    frac_f[1] = rtb_f_n;
    frac_f[2] = rtb_LowAltitudeScaleLength;
    bpIndex_f[0] = rtb_k_b;
    bpIndex_f[1] = rtb_k_d;
    bpIndex_f[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_f, frac_f, Simulator_ConstP.pooled38,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S104>/Add' incorporates:
     *  Sum: '<S104>/Subtract'
     */
    rtb_jxi += rtb_ixk - rtb_jxi;

    /* Interpolation_n-D generated from: '<S102>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S102>/Constant'
     *  Constant: '<S102>/Constant3'
     */
    frac_g[0] = rtb_InterpolationUsingPrelookup;
    frac_g[1] = 0.0;
    frac_g[2] = rtb_LowAltitudeScaleLength;
    bpIndex_g[0] = rtb_k_b;
    bpIndex_g[1] = 0U;
    bpIndex_g[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_g, frac_g, Simulator_ConstP.pooled39,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S102>/Interpolation Using Prelookup' */
    frac_h[0] = rtb_InterpolationUsingPrelookup;
    frac_h[1] = rtb_f_n;
    frac_h[2] = rtb_LowAltitudeScaleLength;
    bpIndex_h[0] = rtb_k_b;
    bpIndex_h[1] = rtb_k_d;
    bpIndex_h[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_h, frac_h, Simulator_ConstP.pooled39,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S102>/Add' incorporates:
     *  Product: '<S102>/Product'
     *  Sum: '<S102>/Subtract'
     */
    rtb_kxj = (rtb_kxj - rtb_ixk) * rtb_Sideslip + rtb_ixk;

    /* Interpolation_n-D generated from: '<S105>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S105>/Constant'
     *  Constant: '<S105>/Constant3'
     */
    frac_i[0] = rtb_InterpolationUsingPrelookup;
    frac_i[1] = 0.0;
    frac_i[2] = rtb_LowAltitudeScaleLength;
    bpIndex_i[0] = rtb_k_b;
    bpIndex_i[1] = 0U;
    bpIndex_i[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_i, frac_i, Simulator_ConstP.pooled40,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S105>/Interpolation Using Prelookup' */
    frac_j[0] = rtb_InterpolationUsingPrelookup;
    frac_j[1] = rtb_f_n;
    frac_j[2] = rtb_LowAltitudeScaleLength;
    bpIndex_j[0] = rtb_k_b;
    bpIndex_j[1] = rtb_k_d;
    bpIndex_j[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_j, frac_j, Simulator_ConstP.pooled40,
      Simulator_ConstP.pooled91);

    /* Sum: '<S105>/Add' incorporates:
     *  Sum: '<S105>/Subtract'
     */
    rtb_ixk += rtb_thetadot - rtb_ixk;

    /* Interpolation_n-D generated from: '<S106>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S106>/Constant'
     *  Constant: '<S106>/Constant3'
     */
    frac_k[0] = rtb_InterpolationUsingPrelookup;
    frac_k[1] = 0.0;
    frac_k[2] = rtb_LowAltitudeScaleLength;
    bpIndex_k[0] = rtb_k_b;
    bpIndex_k[1] = 0U;
    bpIndex_k[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_k, frac_k, Simulator_ConstP.pooled41,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S106>/Interpolation Using Prelookup' */
    frac_l[0] = rtb_InterpolationUsingPrelookup;
    frac_l[1] = rtb_f_n;
    frac_l[2] = rtb_LowAltitudeScaleLength;
    bpIndex_l[0] = rtb_k_b;
    bpIndex_l[1] = rtb_k_d;
    bpIndex_l[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_l, frac_l, Simulator_ConstP.pooled41,
      Simulator_ConstP.pooled91);

    /* Sum: '<S106>/Add' incorporates:
     *  Product: '<S106>/Product'
     *  Sum: '<S106>/Subtract'
     */
    rtb_thetadot += (rtb_Fcn1_f - rtb_thetadot) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S107>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S107>/Constant'
     *  Constant: '<S107>/Constant3'
     */
    frac_m[0] = rtb_InterpolationUsingPrelookup;
    frac_m[1] = 0.0;
    frac_m[2] = rtb_LowAltitudeScaleLength;
    bpIndex_m[0] = rtb_k_b;
    bpIndex_m[1] = 0U;
    bpIndex_m[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_m, frac_m, Simulator_ConstP.pooled42,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S107>/Interpolation Using Prelookup' */
    frac_n[0] = rtb_InterpolationUsingPrelookup;
    frac_n[1] = rtb_f_n;
    frac_n[2] = rtb_LowAltitudeScaleLength;
    bpIndex_n[0] = rtb_k_b;
    bpIndex_n[1] = rtb_k_d;
    bpIndex_n[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_n, frac_n, Simulator_ConstP.pooled42,
                          Simulator_ConstP.pooled91);

    /* Sum: '<S107>/Add' incorporates:
     *  Sum: '<S107>/Subtract'
     */
    rtb_Fcn1_f += rtb_Fcn_m - rtb_Fcn1_f;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Constant: '<S74>/Constant3' */
      Simulator_B.Constant3 = 0.0;

      /* Saturate: '<S68>/Saturation' */
      rtb_Saturation[0] = rtb_ail;
      rtb_Saturation[1] = rtb_elev;
      rtb_Saturation[2] = rtb_rud;
      rtb_Saturation[3] = rtb_flaps;
      rtb_Saturation[4] = rtb_ailOut;

      /* Constant: '<S74>/Constant2' */
      Simulator_B.Constant2 = 0.0;
      for (i = 0; i < 5; i++) {
        /* DiscreteIntegrator: '<S81>/Integrator' */
        if (Simulator_DW.Integrator_PrevResetState != 0) {
          Simulator_DW.Integrator_DSTATE[i] = Simulator_B.Constant3;
        }

        /* Gain: '<S74>/Gain3' incorporates:
         *  DiscreteIntegrator: '<S81>/Integrator'
         */
        rtb_ail = 986.96044010893581 * Simulator_DW.Integrator_DSTATE[i];

        /* DiscreteIntegrator: '<S82>/Integrator' */
        if (Simulator_DW.Integrator_PrevResetState_c != 0) {
          Simulator_DW.Integrator_DSTATE_m[i] = Simulator_B.Constant2;
        }

        rtb_Integrator[i] = Simulator_DW.Integrator_DSTATE_m[i];

        /* End of DiscreteIntegrator: '<S82>/Integrator' */

        /* Sum: '<S74>/Sum' incorporates:
         *  Gain: '<S74>/Gain'
         */
        rtb_Sum_b[i] = (rtb_Saturation[i] - rtb_ail) - 44.422120121759676 *
          rtb_Integrator[i];

        /* Sum: '<S68>/Add1' incorporates:
         *  Gain: '<S68>/Gain3'
         *  Sum: '<S74>/Sum'
         */
        Simulator_B.Add1[i] = 0.0 * rtb_Saturation[i] + rtb_ail;
      }
    }

    /* Product: '<S96>/Product' incorporates:
     *  Gain: '<S101>/CL to Cz'
     */
    rtb_Product_cl[0] = rtb_pgw_p_idx_0 * Simulator_B.Add1[0];
    rtb_Product_cl[1] = rtb_jxi * Simulator_B.Add1[0];
    rtb_Product_cl[2] = -rtb_kxj * Simulator_B.Add1[0];
    rtb_Product_cl[3] = rtb_ixk * Simulator_B.Add1[0];
    rtb_Product_cl[4] = rtb_thetadot * Simulator_B.Add1[0];
    rtb_Product_cl[5] = rtb_Fcn1_f * Simulator_B.Add1[0];

    /* Interpolation_n-D generated from: '<S110>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S110>/Constant'
     *  Constant: '<S110>/Constant3'
     */
    frac_o[0] = rtb_InterpolationUsingPrelookup;
    frac_o[1] = 0.0;
    frac_o[2] = rtb_LowAltitudeScaleLength;
    bpIndex_o[0] = rtb_k_b;
    bpIndex_o[1] = 0U;
    bpIndex_o[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_o, frac_o, Simulator_ConstP.pooled46,
                          Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S110>/Interpolation Using Prelookup' */
    frac_p[0] = rtb_InterpolationUsingPrelookup;
    frac_p[1] = rtb_f_n;
    frac_p[2] = rtb_LowAltitudeScaleLength;
    bpIndex_p[0] = rtb_k_b;
    bpIndex_p[1] = rtb_k_d;
    bpIndex_p[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_p, frac_p, Simulator_ConstP.pooled46,
      Simulator_ConstP.pooled91);

    /* Sum: '<S110>/Add' incorporates:
     *  Sum: '<S110>/Subtract'
     */
    rtb_Fcn_m += rtb_Fcn1_f - rtb_Fcn_m;

    /* Interpolation_n-D generated from: '<S111>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S111>/Constant'
     *  Constant: '<S111>/Constant3'
     */
    frac_q[0] = rtb_InterpolationUsingPrelookup;
    frac_q[1] = 0.0;
    frac_q[2] = rtb_LowAltitudeScaleLength;
    bpIndex_q[0] = rtb_k_b;
    bpIndex_q[1] = 0U;
    bpIndex_q[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_q, frac_q, Simulator_ConstP.pooled47,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S111>/Interpolation Using Prelookup' */
    frac_r[0] = rtb_InterpolationUsingPrelookup;
    frac_r[1] = rtb_f_n;
    frac_r[2] = rtb_LowAltitudeScaleLength;
    bpIndex_r[0] = rtb_k_b;
    bpIndex_r[1] = rtb_k_d;
    bpIndex_r[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_r, frac_r, Simulator_ConstP.pooled47,
      Simulator_ConstP.pooled91);

    /* Sum: '<S111>/Add' incorporates:
     *  Product: '<S111>/Product'
     *  Sum: '<S111>/Subtract'
     */
    rtb_Fcn1_f += (rtb_thetadot - rtb_Fcn1_f) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S109>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S109>/Constant'
     *  Constant: '<S109>/Constant3'
     */
    frac_s[0] = rtb_InterpolationUsingPrelookup;
    frac_s[1] = 0.0;
    frac_s[2] = rtb_LowAltitudeScaleLength;
    bpIndex_s[0] = rtb_k_b;
    bpIndex_s[1] = 0U;
    bpIndex_s[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_s, frac_s, Simulator_ConstP.pooled48,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S109>/Interpolation Using Prelookup' */
    frac_t[0] = rtb_InterpolationUsingPrelookup;
    frac_t[1] = rtb_f_n;
    frac_t[2] = rtb_LowAltitudeScaleLength;
    bpIndex_t[0] = rtb_k_b;
    bpIndex_t[1] = rtb_k_d;
    bpIndex_t[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_t, frac_t, Simulator_ConstP.pooled48,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S109>/Subtract' */
    rtb_ail = rtb_kxj - rtb_thetadot;

    /* Interpolation_n-D generated from: '<S112>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S112>/Constant'
     *  Constant: '<S112>/Constant3'
     */
    frac_u[0] = rtb_InterpolationUsingPrelookup;
    frac_u[1] = 0.0;
    frac_u[2] = rtb_LowAltitudeScaleLength;
    bpIndex_u[0] = rtb_k_b;
    bpIndex_u[1] = 0U;
    bpIndex_u[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_u, frac_u, Simulator_ConstP.pooled49,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S112>/Interpolation Using Prelookup' */
    frac_v[0] = rtb_InterpolationUsingPrelookup;
    frac_v[1] = rtb_f_n;
    frac_v[2] = rtb_LowAltitudeScaleLength;
    bpIndex_v[0] = rtb_k_b;
    bpIndex_v[1] = rtb_k_d;
    bpIndex_v[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_v, frac_v, Simulator_ConstP.pooled49,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S112>/Add' incorporates:
     *  Product: '<S112>/Product'
     *  Sum: '<S112>/Subtract'
     */
    rtb_kxj += (rtb_ixk - rtb_kxj) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S113>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S113>/Constant'
     *  Constant: '<S113>/Constant3'
     */
    frac_w[0] = rtb_InterpolationUsingPrelookup;
    frac_w[1] = 0.0;
    frac_w[2] = rtb_LowAltitudeScaleLength;
    bpIndex_w[0] = rtb_k_b;
    bpIndex_w[1] = 0U;
    bpIndex_w[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_w, frac_w, Simulator_ConstP.pooled50,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S113>/Interpolation Using Prelookup' */
    frac_x[0] = rtb_InterpolationUsingPrelookup;
    frac_x[1] = rtb_f_n;
    frac_x[2] = rtb_LowAltitudeScaleLength;
    bpIndex_x[0] = rtb_k_b;
    bpIndex_x[1] = rtb_k_d;
    bpIndex_x[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_x, frac_x, Simulator_ConstP.pooled50,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S113>/Add' incorporates:
     *  Sum: '<S113>/Subtract'
     */
    rtb_ixk += rtb_jxi - rtb_ixk;

    /* Interpolation_n-D generated from: '<S114>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S114>/Constant'
     *  Constant: '<S114>/Constant3'
     */
    frac_y[0] = rtb_InterpolationUsingPrelookup;
    frac_y[1] = 0.0;
    frac_y[2] = rtb_LowAltitudeScaleLength;
    bpIndex_y[0] = rtb_k_b;
    bpIndex_y[1] = 0U;
    bpIndex_y[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_y, frac_y, Simulator_ConstP.pooled51,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S114>/Interpolation Using Prelookup' */
    frac_z[0] = rtb_InterpolationUsingPrelookup;
    frac_z[1] = rtb_f_n;
    frac_z[2] = rtb_LowAltitudeScaleLength;
    bpIndex_z[0] = rtb_k_b;
    bpIndex_z[1] = rtb_k_d;
    bpIndex_z[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_z, frac_z, Simulator_ConstP.pooled51,
      Simulator_ConstP.pooled91);

    /* Sum: '<S114>/Add' incorporates:
     *  Product: '<S114>/Product'
     *  Sum: '<S114>/Subtract'
     */
    rtb_jxi += (rtb_pgw_p_idx_0 - rtb_jxi) * rtb_Sideslip;

    /* Product: '<S97>/Product' incorporates:
     *  Gain: '<S108>/CL to Cz'
     *  Product: '<S109>/Product'
     *  Sum: '<S109>/Add'
     */
    rtb_elev = rtb_Fcn_m * Simulator_B.Add1[1];
    rtb_rud = rtb_Fcn1_f * Simulator_B.Add1[1];
    rtb_flaps = -(rtb_thetadot + rtb_ail) * Simulator_B.Add1[1];
    rtb_ailOut = rtb_kxj * Simulator_B.Add1[1];
    rtb_Product_d2_idx_4 = rtb_ixk * Simulator_B.Add1[1];
    rtb_Product_d2_idx_5 = rtb_jxi * Simulator_B.Add1[1];

    /* Interpolation_n-D generated from: '<S131>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S131>/Constant'
     *  Constant: '<S131>/Constant3'
     */
    frac_10[0] = rtb_InterpolationUsingPrelookup;
    frac_10[1] = 0.0;
    frac_10[2] = rtb_LowAltitudeScaleLength;
    bpIndex_10[0] = rtb_k_b;
    bpIndex_10[1] = 0U;
    bpIndex_10[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_10, frac_10, Simulator_ConstP.pooled52,
                          Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S131>/Interpolation Using Prelookup' */
    frac_11[0] = rtb_InterpolationUsingPrelookup;
    frac_11[1] = rtb_f_n;
    frac_11[2] = rtb_LowAltitudeScaleLength;
    bpIndex_11[0] = rtb_k_b;
    bpIndex_11[1] = rtb_k_d;
    bpIndex_11[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_11, frac_11, Simulator_ConstP.pooled52,
      Simulator_ConstP.pooled91);

    /* Sum: '<S131>/Add' incorporates:
     *  Product: '<S131>/Product'
     *  Sum: '<S131>/Subtract'
     */
    rtb_Fcn_m += (rtb_Fcn1_f - rtb_Fcn_m) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S132>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S132>/Constant'
     *  Constant: '<S132>/Constant3'
     */
    frac_12[0] = rtb_InterpolationUsingPrelookup;
    frac_12[1] = 0.0;
    frac_12[2] = rtb_LowAltitudeScaleLength;
    bpIndex_12[0] = rtb_k_b;
    bpIndex_12[1] = 0U;
    bpIndex_12[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_12, frac_12, Simulator_ConstP.pooled53,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S132>/Interpolation Using Prelookup' */
    frac_13[0] = rtb_InterpolationUsingPrelookup;
    frac_13[1] = rtb_f_n;
    frac_13[2] = rtb_LowAltitudeScaleLength;
    bpIndex_13[0] = rtb_k_b;
    bpIndex_13[1] = rtb_k_d;
    bpIndex_13[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_13, frac_13, Simulator_ConstP.pooled53,
      Simulator_ConstP.pooled91);

    /* Sum: '<S132>/Add' incorporates:
     *  Sum: '<S132>/Subtract'
     */
    rtb_Fcn1_f += rtb_thetadot - rtb_Fcn1_f;

    /* Interpolation_n-D generated from: '<S130>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S130>/Constant'
     *  Constant: '<S130>/Constant3'
     */
    frac_14[0] = rtb_InterpolationUsingPrelookup;
    frac_14[1] = 0.0;
    frac_14[2] = rtb_LowAltitudeScaleLength;
    bpIndex_14[0] = rtb_k_b;
    bpIndex_14[1] = 0U;
    bpIndex_14[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_14, frac_14, Simulator_ConstP.pooled54,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S130>/Interpolation Using Prelookup' */
    frac_15[0] = rtb_InterpolationUsingPrelookup;
    frac_15[1] = rtb_f_n;
    frac_15[2] = rtb_LowAltitudeScaleLength;
    bpIndex_15[0] = rtb_k_b;
    bpIndex_15[1] = rtb_k_d;
    bpIndex_15[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_15, frac_15, Simulator_ConstP.pooled54,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S130>/Subtract' */
    rtb_ail = rtb_kxj - rtb_thetadot;

    /* Interpolation_n-D generated from: '<S133>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S133>/Constant'
     *  Constant: '<S133>/Constant3'
     */
    frac_16[0] = rtb_InterpolationUsingPrelookup;
    frac_16[1] = 0.0;
    frac_16[2] = rtb_LowAltitudeScaleLength;
    bpIndex_16[0] = rtb_k_b;
    bpIndex_16[1] = 0U;
    bpIndex_16[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_16, frac_16, Simulator_ConstP.pooled55,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S133>/Interpolation Using Prelookup' */
    frac_17[0] = rtb_InterpolationUsingPrelookup;
    frac_17[1] = rtb_f_n;
    frac_17[2] = rtb_LowAltitudeScaleLength;
    bpIndex_17[0] = rtb_k_b;
    bpIndex_17[1] = rtb_k_d;
    bpIndex_17[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_17, frac_17, Simulator_ConstP.pooled55,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S133>/Add' incorporates:
     *  Sum: '<S133>/Subtract'
     */
    rtb_kxj += rtb_ixk - rtb_kxj;

    /* Interpolation_n-D generated from: '<S134>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S134>/Constant'
     *  Constant: '<S134>/Constant3'
     */
    frac_18[0] = rtb_InterpolationUsingPrelookup;
    frac_18[1] = 0.0;
    frac_18[2] = rtb_LowAltitudeScaleLength;
    bpIndex_18[0] = rtb_k_b;
    bpIndex_18[1] = 0U;
    bpIndex_18[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_18, frac_18, Simulator_ConstP.pooled56,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S134>/Interpolation Using Prelookup' */
    frac_19[0] = rtb_InterpolationUsingPrelookup;
    frac_19[1] = rtb_f_n;
    frac_19[2] = rtb_LowAltitudeScaleLength;
    bpIndex_19[0] = rtb_k_b;
    bpIndex_19[1] = rtb_k_d;
    bpIndex_19[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_19, frac_19, Simulator_ConstP.pooled56,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S134>/Add' incorporates:
     *  Product: '<S134>/Product'
     *  Sum: '<S134>/Subtract'
     */
    rtb_ixk += (rtb_jxi - rtb_ixk) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S135>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S135>/Constant'
     *  Constant: '<S135>/Constant3'
     */
    frac_1a[0] = rtb_InterpolationUsingPrelookup;
    frac_1a[1] = 0.0;
    frac_1a[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1a[0] = rtb_k_b;
    bpIndex_1a[1] = 0U;
    bpIndex_1a[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_1a, frac_1a, Simulator_ConstP.pooled57,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S135>/Interpolation Using Prelookup' */
    frac_1b[0] = rtb_InterpolationUsingPrelookup;
    frac_1b[1] = rtb_f_n;
    frac_1b[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1b[0] = rtb_k_b;
    bpIndex_1b[1] = rtb_k_d;
    bpIndex_1b[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_1b, frac_1b, Simulator_ConstP.pooled57,
      Simulator_ConstP.pooled91);

    /* Sum: '<S135>/Add' incorporates:
     *  Sum: '<S135>/Subtract'
     */
    rtb_jxi += rtb_pgw_p_idx_0 - rtb_jxi;

    /* Product: '<S100>/Product' incorporates:
     *  Gain: '<S129>/CL to Cz'
     *  Product: '<S130>/Product'
     *  Sum: '<S130>/Add'
     */
    rtb_Product_ee_idx_0 = rtb_Fcn_m * Simulator_B.Add1[2];
    rtb_Product_ee_idx_1 = rtb_Fcn1_f * Simulator_B.Add1[2];
    rtb_Product_ee_idx_2 = -(rtb_ail * rtb_Sideslip + rtb_thetadot) *
      Simulator_B.Add1[2];
    rtb_Product_ee_idx_3 = rtb_kxj * Simulator_B.Add1[2];
    rtb_Product_ee_idx_4 = rtb_ixk * Simulator_B.Add1[2];
    rtb_Product_ee_idx_5 = rtb_jxi * Simulator_B.Add1[2];

    /* Interpolation_n-D generated from: '<S117>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S117>/Constant'
     *  Constant: '<S117>/Constant3'
     */
    frac_1c[0] = rtb_InterpolationUsingPrelookup;
    frac_1c[1] = 0.0;
    frac_1c[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1c[0] = rtb_k_b;
    bpIndex_1c[1] = 0U;
    bpIndex_1c[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_1c, frac_1c, Simulator_ConstP.pooled58,
                          Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S117>/Interpolation Using Prelookup' */
    frac_1d[0] = rtb_InterpolationUsingPrelookup;
    frac_1d[1] = rtb_f_n;
    frac_1d[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1d[0] = rtb_k_b;
    bpIndex_1d[1] = rtb_k_d;
    bpIndex_1d[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_1d, frac_1d, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Sum: '<S117>/Add' incorporates:
     *  Sum: '<S117>/Subtract'
     */
    rtb_Fcn_m += rtb_Fcn1_f - rtb_Fcn_m;

    /* Interpolation_n-D generated from: '<S118>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S118>/Constant'
     *  Constant: '<S118>/Constant3'
     */
    frac_1e[0] = rtb_InterpolationUsingPrelookup;
    frac_1e[1] = 0.0;
    frac_1e[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1e[0] = rtb_k_b;
    bpIndex_1e[1] = 0U;
    bpIndex_1e[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_1e, frac_1e, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S118>/Interpolation Using Prelookup' */
    frac_1f[0] = rtb_InterpolationUsingPrelookup;
    frac_1f[1] = rtb_f_n;
    frac_1f[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1f[0] = rtb_k_b;
    bpIndex_1f[1] = rtb_k_d;
    bpIndex_1f[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_1f, frac_1f, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Sum: '<S118>/Add' incorporates:
     *  Product: '<S118>/Product'
     *  Sum: '<S118>/Subtract'
     */
    rtb_Fcn1_f += (rtb_thetadot - rtb_Fcn1_f) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S116>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S116>/Constant'
     *  Constant: '<S116>/Constant3'
     */
    frac_1g[0] = rtb_InterpolationUsingPrelookup;
    frac_1g[1] = 0.0;
    frac_1g[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1g[0] = rtb_k_b;
    bpIndex_1g[1] = 0U;
    bpIndex_1g[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_1g, frac_1g, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S116>/Interpolation Using Prelookup' */
    frac_1h[0] = rtb_InterpolationUsingPrelookup;
    frac_1h[1] = rtb_f_n;
    frac_1h[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1h[0] = rtb_k_b;
    bpIndex_1h[1] = rtb_k_d;
    bpIndex_1h[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_1h, frac_1h, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S116>/Subtract' */
    rtb_ail = rtb_kxj - rtb_thetadot;

    /* Interpolation_n-D generated from: '<S119>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S119>/Constant'
     *  Constant: '<S119>/Constant3'
     */
    frac_1i[0] = rtb_InterpolationUsingPrelookup;
    frac_1i[1] = 0.0;
    frac_1i[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1i[0] = rtb_k_b;
    bpIndex_1i[1] = 0U;
    bpIndex_1i[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_1i, frac_1i, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S119>/Interpolation Using Prelookup' */
    frac_1j[0] = rtb_InterpolationUsingPrelookup;
    frac_1j[1] = rtb_f_n;
    frac_1j[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1j[0] = rtb_k_b;
    bpIndex_1j[1] = rtb_k_d;
    bpIndex_1j[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_1j, frac_1j, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S119>/Add' incorporates:
     *  Product: '<S119>/Product'
     *  Sum: '<S119>/Subtract'
     */
    rtb_kxj += (rtb_ixk - rtb_kxj) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S120>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S120>/Constant'
     *  Constant: '<S120>/Constant3'
     */
    frac_1k[0] = rtb_InterpolationUsingPrelookup;
    frac_1k[1] = 0.0;
    frac_1k[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1k[0] = rtb_k_b;
    bpIndex_1k[1] = 0U;
    bpIndex_1k[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_1k, frac_1k, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S120>/Interpolation Using Prelookup' */
    frac_1l[0] = rtb_InterpolationUsingPrelookup;
    frac_1l[1] = rtb_f_n;
    frac_1l[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1l[0] = rtb_k_b;
    bpIndex_1l[1] = rtb_k_d;
    bpIndex_1l[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_1l, frac_1l, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S120>/Add' incorporates:
     *  Sum: '<S120>/Subtract'
     */
    rtb_ixk += rtb_jxi - rtb_ixk;

    /* Interpolation_n-D generated from: '<S121>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S121>/Constant'
     *  Constant: '<S121>/Constant3'
     */
    frac_1m[0] = rtb_InterpolationUsingPrelookup;
    frac_1m[1] = 0.0;
    frac_1m[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1m[0] = rtb_k_b;
    bpIndex_1m[1] = 0U;
    bpIndex_1m[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_1m, frac_1m, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S121>/Interpolation Using Prelookup' */
    frac_1n[0] = rtb_InterpolationUsingPrelookup;
    frac_1n[1] = rtb_f_n;
    frac_1n[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1n[0] = rtb_k_b;
    bpIndex_1n[1] = rtb_k_d;
    bpIndex_1n[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_1n, frac_1n, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Sum: '<S121>/Add' incorporates:
     *  Product: '<S121>/Product'
     *  Sum: '<S121>/Subtract'
     */
    rtb_jxi += (rtb_pgw_p_idx_0 - rtb_jxi) * rtb_Sideslip;

    /* Product: '<S98>/Product' incorporates:
     *  Gain: '<S115>/CL to Cz'
     *  Product: '<S116>/Product'
     *  Sum: '<S116>/Add'
     */
    rtb_Divide_idx_0 = rtb_Fcn_m * Simulator_B.Add1[3];
    rtb_Divide_idx_1 = rtb_Fcn1_f * Simulator_B.Add1[3];
    rtb_Divide_idx_2 = -(rtb_thetadot + rtb_ail) * Simulator_B.Add1[3];
    rtb_Divide_idx_3 = rtb_kxj * Simulator_B.Add1[3];
    rtb_Divide_idx_4 = rtb_ixk * Simulator_B.Add1[3];
    rtb_ail = rtb_jxi * Simulator_B.Add1[3];

    /* Interpolation_n-D generated from: '<S124>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S124>/Constant'
     *  Constant: '<S124>/Constant3'
     */
    frac_1o[0] = rtb_InterpolationUsingPrelookup;
    frac_1o[1] = 0.0;
    frac_1o[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1o[0] = rtb_k_b;
    bpIndex_1o[1] = 0U;
    bpIndex_1o[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_1o, frac_1o, Simulator_ConstP.pooled58,
                          Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S124>/Interpolation Using Prelookup' */
    frac_1p[0] = rtb_InterpolationUsingPrelookup;
    frac_1p[1] = rtb_f_n;
    frac_1p[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1p[0] = rtb_k_b;
    bpIndex_1p[1] = rtb_k_d;
    bpIndex_1p[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_1p, frac_1p, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Sum: '<S124>/Add' incorporates:
     *  Product: '<S124>/Product'
     *  Sum: '<S124>/Subtract'
     */
    rtb_Fcn_m += (rtb_Fcn1_f - rtb_Fcn_m) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S125>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S125>/Constant'
     *  Constant: '<S125>/Constant3'
     */
    frac_1q[0] = rtb_InterpolationUsingPrelookup;
    frac_1q[1] = 0.0;
    frac_1q[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1q[0] = rtb_k_b;
    bpIndex_1q[1] = 0U;
    bpIndex_1q[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_1q, frac_1q, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S125>/Interpolation Using Prelookup' */
    frac_1r[0] = rtb_InterpolationUsingPrelookup;
    frac_1r[1] = rtb_f_n;
    frac_1r[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1r[0] = rtb_k_b;
    bpIndex_1r[1] = rtb_k_d;
    bpIndex_1r[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_1r, frac_1r, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Sum: '<S125>/Add' incorporates:
     *  Sum: '<S125>/Subtract'
     */
    rtb_Fcn1_f += rtb_thetadot - rtb_Fcn1_f;

    /* Interpolation_n-D generated from: '<S123>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S123>/Constant'
     *  Constant: '<S123>/Constant3'
     */
    frac_1s[0] = rtb_InterpolationUsingPrelookup;
    frac_1s[1] = 0.0;
    frac_1s[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1s[0] = rtb_k_b;
    bpIndex_1s[1] = 0U;
    bpIndex_1s[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_1s, frac_1s, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S123>/Interpolation Using Prelookup' */
    frac_1t[0] = rtb_InterpolationUsingPrelookup;
    frac_1t[1] = rtb_f_n;
    frac_1t[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1t[0] = rtb_k_b;
    bpIndex_1t[1] = rtb_k_d;
    bpIndex_1t[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_1t, frac_1t, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Gain: '<S122>/CL to Cz' incorporates:
     *  Product: '<S123>/Product'
     *  Sum: '<S123>/Add'
     *  Sum: '<S123>/Subtract'
     */
    rtb_thetadot = -((rtb_kxj - rtb_thetadot) * rtb_Sideslip + rtb_thetadot);

    /* Interpolation_n-D generated from: '<S126>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S126>/Constant'
     *  Constant: '<S126>/Constant3'
     */
    frac_1u[0] = rtb_InterpolationUsingPrelookup;
    frac_1u[1] = 0.0;
    frac_1u[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1u[0] = rtb_k_b;
    bpIndex_1u[1] = 0U;
    bpIndex_1u[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_1u, frac_1u, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S126>/Interpolation Using Prelookup' */
    frac_1v[0] = rtb_InterpolationUsingPrelookup;
    frac_1v[1] = rtb_f_n;
    frac_1v[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1v[0] = rtb_k_b;
    bpIndex_1v[1] = rtb_k_d;
    bpIndex_1v[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_1v, frac_1v, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S126>/Add' incorporates:
     *  Sum: '<S126>/Subtract'
     */
    rtb_kxj += rtb_ixk - rtb_kxj;

    /* Interpolation_n-D generated from: '<S127>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S127>/Constant'
     *  Constant: '<S127>/Constant3'
     */
    frac_1w[0] = rtb_InterpolationUsingPrelookup;
    frac_1w[1] = 0.0;
    frac_1w[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1w[0] = rtb_k_b;
    bpIndex_1w[1] = 0U;
    bpIndex_1w[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_1w, frac_1w, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S127>/Interpolation Using Prelookup' */
    frac_1x[0] = rtb_InterpolationUsingPrelookup;
    frac_1x[1] = rtb_f_n;
    frac_1x[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1x[0] = rtb_k_b;
    bpIndex_1x[1] = rtb_k_d;
    bpIndex_1x[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_1x, frac_1x, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S127>/Add' incorporates:
     *  Product: '<S127>/Product'
     *  Sum: '<S127>/Subtract'
     */
    rtb_ixk += (rtb_jxi - rtb_ixk) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S128>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S128>/Constant'
     *  Constant: '<S128>/Constant3'
     */
    frac_1y[0] = rtb_InterpolationUsingPrelookup;
    frac_1y[1] = 0.0;
    frac_1y[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1y[0] = rtb_k_b;
    bpIndex_1y[1] = 0U;
    bpIndex_1y[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_1y, frac_1y, Simulator_ConstP.pooled58,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S128>/Interpolation Using Prelookup' */
    frac_1z[0] = rtb_InterpolationUsingPrelookup;
    frac_1z[1] = rtb_f_n;
    frac_1z[2] = rtb_LowAltitudeScaleLength;
    bpIndex_1z[0] = rtb_k_b;
    bpIndex_1z[1] = rtb_k_d;
    bpIndex_1z[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_1z, frac_1z, Simulator_ConstP.pooled58,
      Simulator_ConstP.pooled91);

    /* Sum: '<S128>/Add' incorporates:
     *  Sum: '<S128>/Subtract'
     */
    rtb_jxi += rtb_pgw_p_idx_0 - rtb_jxi;

    /* Sum: '<S94>/Sum' incorporates:
     *  Product: '<S99>/Product'
     */
    rtb_Product_ee_idx_0 = (((rtb_Product_cl[0] + rtb_elev) +
      rtb_Product_ee_idx_0) + rtb_Divide_idx_0) + rtb_Fcn_m * Simulator_B.Add1[4];
    rtb_Product_ee_idx_1 = (((rtb_Product_cl[1] + rtb_rud) +
      rtb_Product_ee_idx_1) + rtb_Divide_idx_1) + rtb_Fcn1_f * Simulator_B.Add1
      [4];
    rtb_Product_ee_idx_2 = (((rtb_Product_cl[2] + rtb_flaps) +
      rtb_Product_ee_idx_2) + rtb_Divide_idx_2) + rtb_thetadot *
      Simulator_B.Add1[4];
    rtb_Product_ee_idx_3 = (((rtb_Product_cl[3] + rtb_ailOut) +
      rtb_Product_ee_idx_3) + rtb_Divide_idx_3) + rtb_kxj * Simulator_B.Add1[4];
    rtb_Product_ee_idx_4 = (((rtb_Product_cl[4] + rtb_Product_d2_idx_4) +
      rtb_Product_ee_idx_4) + rtb_Divide_idx_4) + rtb_ixk * Simulator_B.Add1[4];
    rtb_Product_ee_idx_5 = (((rtb_Product_cl[5] + rtb_Product_d2_idx_5) +
      rtb_Product_ee_idx_5) + rtb_ail) + rtb_jxi * Simulator_B.Add1[4];

    /* Interpolation_n-D generated from: '<S140>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S140>/Constant'
     *  Constant: '<S140>/Constant3'
     */
    frac_20[0] = rtb_InterpolationUsingPrelookup;
    frac_20[1] = 0.0;
    frac_20[2] = rtb_LowAltitudeScaleLength;
    bpIndex_20[0] = rtb_k_b;
    bpIndex_20[1] = 0U;
    bpIndex_20[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_20, frac_20, Simulator_ConstP.pooled59,
                          Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S140>/Interpolation Using Prelookup' */
    frac_21[0] = rtb_InterpolationUsingPrelookup;
    frac_21[1] = rtb_f_n;
    frac_21[2] = rtb_LowAltitudeScaleLength;
    bpIndex_21[0] = rtb_k_b;
    bpIndex_21[1] = rtb_k_d;
    bpIndex_21[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_21, frac_21, Simulator_ConstP.pooled59,
      Simulator_ConstP.pooled91);

    /* Sum: '<S140>/Add' incorporates:
     *  Sum: '<S140>/Subtract'
     */
    rtb_Fcn_m += rtb_Fcn1_f - rtb_Fcn_m;

    /* Interpolation_n-D generated from: '<S141>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S141>/Constant'
     *  Constant: '<S141>/Constant3'
     */
    frac_22[0] = rtb_InterpolationUsingPrelookup;
    frac_22[1] = 0.0;
    frac_22[2] = rtb_LowAltitudeScaleLength;
    bpIndex_22[0] = rtb_k_b;
    bpIndex_22[1] = 0U;
    bpIndex_22[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_22, frac_22, Simulator_ConstP.pooled60,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S141>/Interpolation Using Prelookup' */
    frac_23[0] = rtb_InterpolationUsingPrelookup;
    frac_23[1] = rtb_f_n;
    frac_23[2] = rtb_LowAltitudeScaleLength;
    bpIndex_23[0] = rtb_k_b;
    bpIndex_23[1] = rtb_k_d;
    bpIndex_23[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_23, frac_23, Simulator_ConstP.pooled60,
      Simulator_ConstP.pooled91);

    /* Sum: '<S141>/Add' incorporates:
     *  Product: '<S141>/Product'
     *  Sum: '<S141>/Subtract'
     */
    rtb_Fcn1_f += (rtb_thetadot - rtb_Fcn1_f) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S139>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S139>/Constant'
     *  Constant: '<S139>/Constant3'
     */
    frac_24[0] = rtb_InterpolationUsingPrelookup;
    frac_24[1] = 0.0;
    frac_24[2] = rtb_LowAltitudeScaleLength;
    bpIndex_24[0] = rtb_k_b;
    bpIndex_24[1] = 0U;
    bpIndex_24[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_24, frac_24, Simulator_ConstP.pooled61,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S139>/Interpolation Using Prelookup' */
    frac_25[0] = rtb_InterpolationUsingPrelookup;
    frac_25[1] = rtb_f_n;
    frac_25[2] = rtb_LowAltitudeScaleLength;
    bpIndex_25[0] = rtb_k_b;
    bpIndex_25[1] = rtb_k_d;
    bpIndex_25[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_25, frac_25, Simulator_ConstP.pooled61,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S139>/Subtract' */
    rtb_ail = rtb_kxj - rtb_thetadot;

    /* Interpolation_n-D generated from: '<S142>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S142>/Constant'
     *  Constant: '<S142>/Constant3'
     */
    frac_26[0] = rtb_InterpolationUsingPrelookup;
    frac_26[1] = 0.0;
    frac_26[2] = rtb_LowAltitudeScaleLength;
    bpIndex_26[0] = rtb_k_b;
    bpIndex_26[1] = 0U;
    bpIndex_26[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_26, frac_26, Simulator_ConstP.pooled62,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S142>/Interpolation Using Prelookup' */
    frac_27[0] = rtb_InterpolationUsingPrelookup;
    frac_27[1] = rtb_f_n;
    frac_27[2] = rtb_LowAltitudeScaleLength;
    bpIndex_27[0] = rtb_k_b;
    bpIndex_27[1] = rtb_k_d;
    bpIndex_27[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_27, frac_27, Simulator_ConstP.pooled62,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S142>/Add' incorporates:
     *  Product: '<S142>/Product'
     *  Sum: '<S142>/Subtract'
     */
    rtb_kxj += (rtb_ixk - rtb_kxj) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S143>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S143>/Constant'
     *  Constant: '<S143>/Constant3'
     */
    frac_28[0] = rtb_InterpolationUsingPrelookup;
    frac_28[1] = 0.0;
    frac_28[2] = rtb_LowAltitudeScaleLength;
    bpIndex_28[0] = rtb_k_b;
    bpIndex_28[1] = 0U;
    bpIndex_28[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_28, frac_28, Simulator_ConstP.pooled63,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S143>/Interpolation Using Prelookup' */
    frac_29[0] = rtb_InterpolationUsingPrelookup;
    frac_29[1] = rtb_f_n;
    frac_29[2] = rtb_LowAltitudeScaleLength;
    bpIndex_29[0] = rtb_k_b;
    bpIndex_29[1] = rtb_k_d;
    bpIndex_29[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_29, frac_29, Simulator_ConstP.pooled63,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S143>/Add' incorporates:
     *  Sum: '<S143>/Subtract'
     */
    rtb_ixk += rtb_jxi - rtb_ixk;

    /* Interpolation_n-D generated from: '<S144>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S144>/Constant'
     *  Constant: '<S144>/Constant3'
     */
    frac_2a[0] = rtb_InterpolationUsingPrelookup;
    frac_2a[1] = 0.0;
    frac_2a[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2a[0] = rtb_k_b;
    bpIndex_2a[1] = 0U;
    bpIndex_2a[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_2a, frac_2a, Simulator_ConstP.pooled64,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S144>/Interpolation Using Prelookup' */
    frac_2b[0] = rtb_InterpolationUsingPrelookup;
    frac_2b[1] = rtb_f_n;
    frac_2b[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2b[0] = rtb_k_b;
    bpIndex_2b[1] = rtb_k_d;
    bpIndex_2b[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_2b, frac_2b, Simulator_ConstP.pooled64,
      Simulator_ConstP.pooled91);

    /* Sum: '<S144>/Add' incorporates:
     *  Product: '<S144>/Product'
     *  Sum: '<S144>/Subtract'
     */
    rtb_jxi += (rtb_pgw_p_idx_0 - rtb_jxi) * rtb_Sideslip;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Constant: '<S73>/Constant3' */
      Simulator_B.Constant3_i = 0.0;

      /* DiscreteIntegrator: '<S77>/Integrator' */
      if (Simulator_DW.Integrator_PrevResetState_g != 0) {
        Simulator_DW.Integrator_DSTATE_b[0] = Simulator_B.Constant3_i;
        Simulator_DW.Integrator_DSTATE_b[1] = Simulator_B.Constant3_i;
      }

      /* Gain: '<S73>/Gain3' incorporates:
       *  DiscreteIntegrator: '<S77>/Integrator'
       */
      rtb_pgw_p_idx_0 = 986.96044010893581 * Simulator_DW.Integrator_DSTATE_b[0];
      rtb_elev = 986.96044010893581 * Simulator_DW.Integrator_DSTATE_b[1];

      /* Saturate: '<S68>/Saturation1' */
      if (!(rtb_thrDiff < 0.0)) {
        rtb_Product2_n_idx_1 = rtb_thrDiff;
      }

      /* Constant: '<S73>/Constant2' */
      Simulator_B.Constant2_m = 0.0;

      /* DiscreteIntegrator: '<S78>/Integrator' */
      if (Simulator_DW.Integrator_PrevResetState_l != 0) {
        Simulator_DW.Integrator_DSTATE_c[0] = Simulator_B.Constant2_m;
        Simulator_DW.Integrator_DSTATE_c[1] = Simulator_B.Constant2_m;
      }

      rtb_Integrator_f[0] = Simulator_DW.Integrator_DSTATE_c[0];

      /* Sum: '<S73>/Sum' incorporates:
       *  Gain: '<S73>/Gain'
       *  Saturate: '<S68>/Saturation1'
       */
      rtb_Sum_f[0] = (rtb_thr - rtb_pgw_p_idx_0) - 44.422120121759676 *
        rtb_Integrator_f[0];

      /* DiscreteIntegrator: '<S78>/Integrator' */
      rtb_Integrator_f[1] = Simulator_DW.Integrator_DSTATE_c[1];

      /* Sum: '<S73>/Sum' incorporates:
       *  Gain: '<S73>/Gain'
       */
      rtb_Sum_f[1] = (rtb_Product2_n_idx_1 - rtb_elev) - 44.422120121759676 *
        rtb_Integrator_f[1];

      /* Sum: '<S68>/Add' incorporates:
       *  Gain: '<S68>/Gain1'
       *  Saturate: '<S68>/Saturation1'
       */
      Simulator_B.Add_j[0] = 0.0 * rtb_thr + rtb_pgw_p_idx_0;
      Simulator_B.Add_j[1] = 0.0 * rtb_Product2_n_idx_1 + rtb_elev;
    }

    /* Product: '<S136>/Product' incorporates:
     *  Gain: '<S138>/CL to Cz'
     *  Product: '<S139>/Product'
     *  Sum: '<S139>/Add'
     */
    rtb_Product_cl[0] = rtb_Fcn_m * Simulator_B.Add_j[0];
    rtb_Product_cl[1] = rtb_Fcn1_f * Simulator_B.Add_j[0];
    rtb_Product_cl[2] = -(rtb_thetadot + rtb_ail) * Simulator_B.Add_j[0];
    rtb_Product_cl[3] = rtb_kxj * Simulator_B.Add_j[0];
    rtb_Product_cl[4] = rtb_ixk * Simulator_B.Add_j[0];
    rtb_Product_cl[5] = rtb_jxi * Simulator_B.Add_j[0];

    /* Interpolation_n-D generated from: '<S147>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S147>/Constant'
     *  Constant: '<S147>/Constant3'
     */
    frac_2c[0] = rtb_InterpolationUsingPrelookup;
    frac_2c[1] = 0.0;
    frac_2c[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2c[0] = rtb_k_b;
    bpIndex_2c[1] = 0U;
    bpIndex_2c[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_2c, frac_2c, Simulator_ConstP.pooled65,
                          Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S147>/Interpolation Using Prelookup' */
    frac_2d[0] = rtb_InterpolationUsingPrelookup;
    frac_2d[1] = rtb_f_n;
    frac_2d[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2d[0] = rtb_k_b;
    bpIndex_2d[1] = rtb_k_d;
    bpIndex_2d[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_2d, frac_2d, Simulator_ConstP.pooled65,
      Simulator_ConstP.pooled91);

    /* Sum: '<S147>/Add' incorporates:
     *  Product: '<S147>/Product'
     *  Sum: '<S147>/Subtract'
     */
    rtb_Fcn_m += (rtb_Fcn1_f - rtb_Fcn_m) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S148>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S148>/Constant'
     *  Constant: '<S148>/Constant3'
     */
    frac_2e[0] = rtb_InterpolationUsingPrelookup;
    frac_2e[1] = 0.0;
    frac_2e[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2e[0] = rtb_k_b;
    bpIndex_2e[1] = 0U;
    bpIndex_2e[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_2e, frac_2e, Simulator_ConstP.pooled66,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S148>/Interpolation Using Prelookup' */
    frac_2f[0] = rtb_InterpolationUsingPrelookup;
    frac_2f[1] = rtb_f_n;
    frac_2f[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2f[0] = rtb_k_b;
    bpIndex_2f[1] = rtb_k_d;
    bpIndex_2f[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_2f, frac_2f, Simulator_ConstP.pooled66,
      Simulator_ConstP.pooled91);

    /* Sum: '<S148>/Add' incorporates:
     *  Sum: '<S148>/Subtract'
     */
    rtb_Fcn1_f += rtb_thetadot - rtb_Fcn1_f;

    /* Interpolation_n-D generated from: '<S146>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S146>/Constant'
     *  Constant: '<S146>/Constant3'
     */
    frac_2g[0] = rtb_InterpolationUsingPrelookup;
    frac_2g[1] = 0.0;
    frac_2g[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2g[0] = rtb_k_b;
    bpIndex_2g[1] = 0U;
    bpIndex_2g[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_2g, frac_2g, Simulator_ConstP.pooled67,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S146>/Interpolation Using Prelookup' */
    frac_2h[0] = rtb_InterpolationUsingPrelookup;
    frac_2h[1] = rtb_f_n;
    frac_2h[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2h[0] = rtb_k_b;
    bpIndex_2h[1] = rtb_k_d;
    bpIndex_2h[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_2h, frac_2h, Simulator_ConstP.pooled67,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S146>/Add' incorporates:
     *  Product: '<S146>/Product'
     *  Sum: '<S146>/Subtract'
     */
    rtb_thrDiff = (rtb_kxj - rtb_thetadot) * rtb_Sideslip + rtb_thetadot;

    /* Interpolation_n-D generated from: '<S149>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S149>/Constant'
     *  Constant: '<S149>/Constant3'
     */
    frac_2i[0] = rtb_InterpolationUsingPrelookup;
    frac_2i[1] = 0.0;
    frac_2i[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2i[0] = rtb_k_b;
    bpIndex_2i[1] = 0U;
    bpIndex_2i[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_2i, frac_2i, Simulator_ConstP.pooled68,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S149>/Interpolation Using Prelookup' */
    frac_2j[0] = rtb_InterpolationUsingPrelookup;
    frac_2j[1] = rtb_f_n;
    frac_2j[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2j[0] = rtb_k_b;
    bpIndex_2j[1] = rtb_k_d;
    bpIndex_2j[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_2j, frac_2j, Simulator_ConstP.pooled68,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S149>/Add' incorporates:
     *  Sum: '<S149>/Subtract'
     */
    rtb_kxj += rtb_ixk - rtb_kxj;

    /* Interpolation_n-D generated from: '<S150>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S150>/Constant'
     *  Constant: '<S150>/Constant3'
     */
    frac_2k[0] = rtb_InterpolationUsingPrelookup;
    frac_2k[1] = 0.0;
    frac_2k[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2k[0] = rtb_k_b;
    bpIndex_2k[1] = 0U;
    bpIndex_2k[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_2k, frac_2k, Simulator_ConstP.pooled69,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S150>/Interpolation Using Prelookup' */
    frac_2l[0] = rtb_InterpolationUsingPrelookup;
    frac_2l[1] = rtb_f_n;
    frac_2l[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2l[0] = rtb_k_b;
    bpIndex_2l[1] = rtb_k_d;
    bpIndex_2l[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_2l, frac_2l, Simulator_ConstP.pooled69,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S150>/Add' incorporates:
     *  Product: '<S150>/Product'
     *  Sum: '<S150>/Subtract'
     */
    rtb_ixk += (rtb_jxi - rtb_ixk) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S151>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S151>/Constant'
     *  Constant: '<S151>/Constant3'
     */
    frac_2m[0] = rtb_InterpolationUsingPrelookup;
    frac_2m[1] = 0.0;
    frac_2m[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2m[0] = rtb_k_b;
    bpIndex_2m[1] = 0U;
    bpIndex_2m[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_2m, frac_2m, Simulator_ConstP.pooled70,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S151>/Interpolation Using Prelookup' */
    frac_2n[0] = rtb_InterpolationUsingPrelookup;
    frac_2n[1] = rtb_f_n;
    frac_2n[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2n[0] = rtb_k_b;
    bpIndex_2n[1] = rtb_k_d;
    bpIndex_2n[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_2n, frac_2n, Simulator_ConstP.pooled70,
      Simulator_ConstP.pooled91);

    /* Sum: '<S151>/Add' incorporates:
     *  Sum: '<S151>/Subtract'
     */
    rtb_jxi += rtb_pgw_p_idx_0 - rtb_jxi;

    /* Product: '<S137>/Product' incorporates:
     *  Gain: '<S145>/CL to Cz'
     *  Sum: '<S89>/Add'
     *  Sum: '<S95>/Sum'
     */
    rtb_Sum_a_0[0] = (rtb_Fcn_m * Simulator_B.Add_j[1] + rtb_Product_cl[0]) +
      rtb_Product_ee_idx_0;
    rtb_Sum_a_0[1] = (rtb_Fcn1_f * Simulator_B.Add_j[1] + rtb_Product_cl[1]) +
      rtb_Product_ee_idx_1;
    rtb_Sum_a_0[2] = (-rtb_thrDiff * Simulator_B.Add_j[1] + rtb_Product_cl[2]) +
      rtb_Product_ee_idx_2;
    rtb_Sum_a_0[3] = (rtb_kxj * Simulator_B.Add_j[1] + rtb_Product_cl[3]) +
      rtb_Product_ee_idx_3;
    rtb_Sum_a_0[4] = (rtb_ixk * Simulator_B.Add_j[1] + rtb_Product_cl[4]) +
      rtb_Product_ee_idx_4;
    rtb_Sum_a_0[5] = (rtb_jxi * Simulator_B.Add_j[1] + rtb_Product_cl[5]) +
      rtb_Product_ee_idx_5;

    /* Sum: '<S89>/Add' */
    for (i = 0; i < 6; i++) {
      rtb_Product_cl[i] = rtb_Sum_a_0[i];
    }

    /* Interpolation_n-D generated from: '<S164>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S164>/Constant'
     *  Constant: '<S164>/Constant3'
     */
    frac_2o[0] = rtb_InterpolationUsingPrelookup;
    frac_2o[1] = 0.0;
    frac_2o[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2o[0] = rtb_k_b;
    bpIndex_2o[1] = 0U;
    bpIndex_2o[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_2o, frac_2o, Simulator_ConstP.pooled71,
                          Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S164>/Interpolation Using Prelookup' */
    frac_2p[0] = rtb_InterpolationUsingPrelookup;
    frac_2p[1] = rtb_f_n;
    frac_2p[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2p[0] = rtb_k_b;
    bpIndex_2p[1] = rtb_k_d;
    bpIndex_2p[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_2p, frac_2p, Simulator_ConstP.pooled71,
      Simulator_ConstP.pooled91);

    /* Sum: '<S164>/Add' incorporates:
     *  Product: '<S164>/Product'
     *  Sum: '<S164>/Subtract'
     */
    rtb_Fcn_m += (rtb_Fcn1_f - rtb_Fcn_m) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S165>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S165>/Constant'
     *  Constant: '<S165>/Constant3'
     */
    frac_2q[0] = rtb_InterpolationUsingPrelookup;
    frac_2q[1] = 0.0;
    frac_2q[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2q[0] = rtb_k_b;
    bpIndex_2q[1] = 0U;
    bpIndex_2q[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_2q, frac_2q, Simulator_ConstP.pooled72,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S165>/Interpolation Using Prelookup' */
    frac_2r[0] = rtb_InterpolationUsingPrelookup;
    frac_2r[1] = rtb_f_n;
    frac_2r[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2r[0] = rtb_k_b;
    bpIndex_2r[1] = rtb_k_d;
    bpIndex_2r[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_2r, frac_2r, Simulator_ConstP.pooled72,
      Simulator_ConstP.pooled91);

    /* Sum: '<S165>/Add' incorporates:
     *  Sum: '<S165>/Subtract'
     */
    rtb_Fcn1_f += rtb_thetadot - rtb_Fcn1_f;

    /* Interpolation_n-D generated from: '<S163>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S163>/Constant'
     *  Constant: '<S163>/Constant3'
     */
    frac_2s[0] = rtb_InterpolationUsingPrelookup;
    frac_2s[1] = 0.0;
    frac_2s[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2s[0] = rtb_k_b;
    bpIndex_2s[1] = 0U;
    bpIndex_2s[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_2s, frac_2s, Simulator_ConstP.pooled73,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S163>/Interpolation Using Prelookup' */
    frac_2t[0] = rtb_InterpolationUsingPrelookup;
    frac_2t[1] = rtb_f_n;
    frac_2t[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2t[0] = rtb_k_b;
    bpIndex_2t[1] = rtb_k_d;
    bpIndex_2t[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_2t, frac_2t, Simulator_ConstP.pooled73,
                        Simulator_ConstP.pooled91);

    /* Gain: '<S162>/CL to Cz' incorporates:
     *  Product: '<S163>/Product'
     *  Sum: '<S163>/Add'
     *  Sum: '<S163>/Subtract'
     */
    rtb_thetadot = -((rtb_kxj - rtb_thetadot) * rtb_Sideslip + rtb_thetadot);

    /* Interpolation_n-D generated from: '<S166>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S166>/Constant'
     *  Constant: '<S166>/Constant3'
     */
    frac_2u[0] = rtb_InterpolationUsingPrelookup;
    frac_2u[1] = 0.0;
    frac_2u[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2u[0] = rtb_k_b;
    bpIndex_2u[1] = 0U;
    bpIndex_2u[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_2u, frac_2u, Simulator_ConstP.pooled74,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S166>/Interpolation Using Prelookup' */
    frac_2v[0] = rtb_InterpolationUsingPrelookup;
    frac_2v[1] = rtb_f_n;
    frac_2v[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2v[0] = rtb_k_b;
    bpIndex_2v[1] = rtb_k_d;
    bpIndex_2v[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_2v, frac_2v, Simulator_ConstP.pooled74,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S166>/Add' incorporates:
     *  Sum: '<S166>/Subtract'
     */
    rtb_kxj += rtb_ixk - rtb_kxj;

    /* Interpolation_n-D generated from: '<S167>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S167>/Constant'
     *  Constant: '<S167>/Constant3'
     */
    frac_2w[0] = rtb_InterpolationUsingPrelookup;
    frac_2w[1] = 0.0;
    frac_2w[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2w[0] = rtb_k_b;
    bpIndex_2w[1] = 0U;
    bpIndex_2w[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_2w, frac_2w, Simulator_ConstP.pooled75,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S167>/Interpolation Using Prelookup' */
    frac_2x[0] = rtb_InterpolationUsingPrelookup;
    frac_2x[1] = rtb_f_n;
    frac_2x[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2x[0] = rtb_k_b;
    bpIndex_2x[1] = rtb_k_d;
    bpIndex_2x[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_2x, frac_2x, Simulator_ConstP.pooled75,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S167>/Add' incorporates:
     *  Product: '<S167>/Product'
     *  Sum: '<S167>/Subtract'
     */
    rtb_ixk += (rtb_jxi - rtb_ixk) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S168>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S168>/Constant'
     *  Constant: '<S168>/Constant3'
     */
    frac_2y[0] = rtb_InterpolationUsingPrelookup;
    frac_2y[1] = 0.0;
    frac_2y[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2y[0] = rtb_k_b;
    bpIndex_2y[1] = 0U;
    bpIndex_2y[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_2y, frac_2y, Simulator_ConstP.pooled76,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S168>/Interpolation Using Prelookup' */
    frac_2z[0] = rtb_InterpolationUsingPrelookup;
    frac_2z[1] = rtb_f_n;
    frac_2z[2] = rtb_LowAltitudeScaleLength;
    bpIndex_2z[0] = rtb_k_b;
    bpIndex_2z[1] = rtb_k_d;
    bpIndex_2z[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_2z, frac_2z, Simulator_ConstP.pooled76,
      Simulator_ConstP.pooled91);

    /* Sum: '<S168>/Add' incorporates:
     *  Sum: '<S168>/Subtract'
     */
    rtb_jxi += rtb_pgw_p_idx_0 - rtb_jxi;

    /* Outputs for Enabled SubSystem: '<S261>/Hqgw' incorporates:
     *  EnablePort: '<S273>/Enable'
     */
    /* Outputs for Enabled SubSystem: '<S261>/Hpgw' incorporates:
     *  EnablePort: '<S272>/Enable'
     */
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      if (rtmIsMajorTimeStep((&Simulator_M)) && Simulator_DW.Hpgw_MODE) {
        /* Disable for Outport: '<S272>/pgw' */
        Simulator_B.sigma_w[0] = 0.0;
        Simulator_B.sigma_w[1] = 0.0;
        Simulator_DW.Hpgw_MODE = false;
      }

      if (rtmIsMajorTimeStep((&Simulator_M)) && Simulator_DW.Hqgw_MODE) {
        /* Disable for Outport: '<S273>/qgw' */
        Simulator_B.w_b[0] = 0.0;
        Simulator_B.w_b[1] = 0.0;
        Simulator_DW.Hqgw_MODE = false;
      }
    }

    /* End of Outputs for SubSystem: '<S261>/Hqgw' */
    if (Simulator_DW.Hpgw_MODE) {
      /* Fcn: '<S272>/sqrt(0.8//V)' */
      rtb_pgw_p_idx_0 = 0.8 / rtb_UnitConversion_i;
      if (rtb_pgw_p_idx_0 < 0.0) {
        rtb_pgw_p_idx_0 = -std::sqrt(-rtb_pgw_p_idx_0);
      } else {
        rtb_pgw_p_idx_0 = std::sqrt(rtb_pgw_p_idx_0);
      }

      /* Product: '<S272>/sigma_w' incorporates:
       *  Integrator: '<S272>/pgw_p'
       */
      Simulator_B.sigma_w[0] = 32.808398950131235 * Simulator_X.pgw_p_CSTATE[0];
      Simulator_B.sigma_w[1] = rtb_MediumHighAltitudeIntensity *
        Simulator_X.pgw_p_CSTATE[1];

      /* Product: '<S272>/w3' */
      rtb_thrDiff = rtb_UnitConversion_i * 0.12599440010712751;

      /* Product: '<S272>/w' incorporates:
       *  Fcn: '<S272>/sqrt(0.8//V)'
       *  Integrator: '<S272>/pgw_p'
       *  Math: '<S272>/L^1//3'
       *  Product: '<S272>/Lug//V1'
       *  Product: '<S272>/w1'
       *  Product: '<S272>/w2'
       *  Sum: '<S272>/Sum'
       */
      Simulator_B.w_hh[0] = (rtb_pgw_p_idx_0 / rt_powd_snf(frac[0],
        0.33333333333333331) * 0.70804121802368036 * Simulator_B.Product[3] -
        Simulator_X.pgw_p_CSTATE[0]) * rtb_thrDiff;

      /* Math: '<S272>/L^1//3' incorporates:
       *  Gain: '<S268>/Lw'
       */
      if (Simulator_ConstB.UnitConversion_cm < 0.0) {
        rtb_MediumHighAltitudeIntensity = -rt_powd_snf
          (-Simulator_ConstB.UnitConversion_cm, 0.33333333333333331);
      } else {
        rtb_MediumHighAltitudeIntensity = rt_powd_snf
          (Simulator_ConstB.UnitConversion_cm, 0.33333333333333331);
      }

      /* Product: '<S272>/w' incorporates:
       *  Fcn: '<S272>/sqrt(0.8//V)'
       *  Integrator: '<S272>/pgw_p'
       *  Product: '<S272>/Lug//V1'
       *  Product: '<S272>/w1'
       *  Product: '<S272>/w2'
       *  Sum: '<S272>/Sum'
       */
      Simulator_B.w_hh[1] = (rtb_pgw_p_idx_0 / rtb_MediumHighAltitudeIntensity *
        0.70804121802368036 * Simulator_B.Product[3] - Simulator_X.pgw_p_CSTATE
        [1]) * rtb_thrDiff;
    }

    /* End of Outputs for SubSystem: '<S261>/Hpgw' */

    /* Outputs for Enabled SubSystem: '<S261>/Hqgw' incorporates:
     *  EnablePort: '<S273>/Enable'
     */
    if (Simulator_DW.Hqgw_MODE) {
      /* Gain: '<S273>/pi//4' */
      rtb_thrDiff = 0.78539816339744828 * rtb_UnitConversion_i;

      /* Product: '<S273>/w' incorporates:
       *  Integrator: '<S273>/qgw_p'
       *  Product: '<S273>/wg//V'
       *  Sum: '<S273>/Sum'
       */
      Simulator_B.w_b[0] = (Simulator_B.LwgV1[0] / rtb_UnitConversion_i -
                            Simulator_X.qgw_p_CSTATE[0]) * (rtb_thrDiff /
        6.2335958005249337);
      Simulator_B.w_b[1] = (Simulator_B.LwgV1[1] / rtb_UnitConversion_i -
                            Simulator_X.qgw_p_CSTATE[1]) * (rtb_thrDiff /
        6.2335958005249337);
    }

    /* End of Outputs for SubSystem: '<S261>/Hqgw' */

    /* Outputs for Enabled SubSystem: '<S261>/Hrgw' incorporates:
     *  EnablePort: '<S274>/Enable'
     */
    if ((rtmIsMajorTimeStep((&Simulator_M)) &&
         (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) && rtmIsMajorTimeStep
        ((&Simulator_M)) && Simulator_DW.Hrgw_MODE) {
      /* Disable for Outport: '<S274>/rgw' */
      Simulator_B.UnaryMinus[0] = 0.0;
      Simulator_B.UnaryMinus[1] = 0.0;
      Simulator_DW.Hrgw_MODE = false;
    }

    if (Simulator_DW.Hrgw_MODE) {
      /* Gain: '<S274>/pi//3' */
      rtb_thrDiff = 1.0471975511965976 * rtb_UnitConversion_i;

      /* Product: '<S274>/w' incorporates:
       *  Integrator: '<S274>/rgw_p'
       *  Product: '<S274>/vg//V'
       *  Sum: '<S274>/Sum'
       */
      Simulator_B.w_jk[0] = (Simulator_B.w1[0] / rtb_UnitConversion_i -
        Simulator_X.rgw_p_CSTATE[0]) * (rtb_thrDiff / 6.2335958005249337);

      /* UnaryMinus: '<S274>/Unary Minus' */
      Simulator_B.UnaryMinus[0] = -Simulator_B.w_jk[0];

      /* Product: '<S274>/w' incorporates:
       *  Integrator: '<S274>/rgw_p'
       *  Product: '<S274>/vg//V'
       *  Sum: '<S274>/Sum'
       */
      Simulator_B.w_jk[1] = (Simulator_B.w1[1] / rtb_UnitConversion_i -
        Simulator_X.rgw_p_CSTATE[1]) * (rtb_thrDiff / 6.2335958005249337);

      /* UnaryMinus: '<S274>/Unary Minus' */
      Simulator_B.UnaryMinus[1] = -Simulator_B.w_jk[1];
    }

    /* End of Outputs for SubSystem: '<S261>/Hrgw' */

    /* If: '<S266>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
    if (rtmIsMajorTimeStep((&Simulator_M))) {
      if (rtb_UnitConversion <= 1000.0) {
        rtAction = 0;
      } else if (rtb_UnitConversion >= 2000.0) {
        rtAction = 1;
      } else {
        rtAction = 2;
      }

      Simulator_DW.ifHeightMaxlowaltitudeelseifH_e = rtAction;
    } else {
      rtAction = Simulator_DW.ifHeightMaxlowaltitudeelseifH_e;
    }

    switch (rtAction) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S266>/Low altitude  rates' incorporates:
       *  ActionPort: '<S281>/Action Port'
       */
      /* Sum: '<S287>/Sum' incorporates:
       *  Product: '<S287>/Product1'
       *  Product: '<S287>/Product2'
       */
      frac_0[0] = Simulator_B.sigma_w[0] * 6.123233995736766E-17 -
        Simulator_B.w_b[0];

      /* Sum: '<S287>/Sum1' incorporates:
       *  Product: '<S287>/Product1'
       *  Product: '<S287>/Product2'
       */
      frac_0[1] = Simulator_B.w_b[0] * 6.123233995736766E-17 +
        Simulator_B.sigma_w[0];

      /* Reshape: '<S286>/Reshape1' incorporates:
       *  Product: '<S286>/Product'
       *  SignalConversion generated from: '<S286>/Vector Concatenate'
       */
      for (i = 0; i < 3; i++) {
        rtb_Integrator_o[i] = Simulator_B.VectorConcatenate[i + 6] *
          Simulator_B.UnaryMinus[0] + (Simulator_B.VectorConcatenate[i + 3] *
          frac_0[1] + Simulator_B.VectorConcatenate[i] * frac_0[0]);
      }

      /* End of Reshape: '<S286>/Reshape1' */
      /* End of Outputs for SubSystem: '<S266>/Low altitude  rates' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S266>/Medium//High  altitude rates' incorporates:
       *  ActionPort: '<S282>/Action Port'
       */
      /* Gain: '<S282>/Gain' */
      rtb_Integrator_o[0] = Simulator_B.sigma_w[1];
      rtb_Integrator_o[1] = Simulator_B.w_b[1];
      rtb_Integrator_o[2] = Simulator_B.UnaryMinus[1];

      /* End of Outputs for SubSystem: '<S266>/Medium//High  altitude rates' */
      break;

     case 2:
      /* Outputs for IfAction SubSystem: '<S266>/Interpolate  rates' incorporates:
       *  ActionPort: '<S280>/Action Port'
       */
      /* Sum: '<S285>/Sum' incorporates:
       *  Product: '<S285>/Product1'
       *  Product: '<S285>/Product2'
       */
      rtb_Integrator_o[0] = Simulator_B.sigma_w[0] * 6.123233995736766E-17 -
        Simulator_B.w_b[0];

      /* Sum: '<S285>/Sum1' incorporates:
       *  Product: '<S285>/Product1'
       *  Product: '<S285>/Product2'
       */
      rtb_Integrator_o[1] = Simulator_B.w_b[0] * 6.123233995736766E-17 +
        Simulator_B.sigma_w[0];

      /* Product: '<S284>/Product' incorporates:
       *  SignalConversion generated from: '<S284>/Vector Concatenate'
       */
      for (i = 0; i < 3; i++) {
        frac_0[i] = Simulator_B.VectorConcatenate[i + 6] *
          Simulator_B.UnaryMinus[0] + (Simulator_B.VectorConcatenate[i + 3] *
          rtb_Integrator_o[1] + Simulator_B.VectorConcatenate[i] *
          rtb_Integrator_o[0]);
      }

      /* End of Product: '<S284>/Product' */

      /* Sum: '<S280>/Sum1' incorporates:
       *  Constant: '<S280>/max_height_low'
       */
      rtb_UnitConversion -= 1000.0;

      /* Sum: '<S280>/Sum3' incorporates:
       *  Product: '<S280>/Product1'
       *  Sum: '<S280>/Sum2'
       */
      rtb_Integrator_o[0] = (Simulator_B.sigma_w[1] - frac_0[0]) *
        rtb_UnitConversion / 1000.0 + frac_0[0];
      rtb_Integrator_o[1] = (Simulator_B.w_b[1] - frac_0[1]) *
        rtb_UnitConversion / 1000.0 + frac_0[1];
      rtb_Integrator_o[2] = (Simulator_B.UnaryMinus[1] - frac_0[2]) *
        rtb_UnitConversion / 1000.0 + frac_0[2];

      /* End of Outputs for SubSystem: '<S266>/Interpolate  rates' */
      break;
    }

    /* End of If: '<S266>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */

    /* Sum: '<S69>/Sum2' incorporates:
     *  Gain: '<S246>/Gain2'
     */
    rtb_thrDiff = 0.0 * rtb_Integrator_o[0] + rtb_Integrator_i[0];
    rtb_UnitConversion = 0.0 * rtb_Integrator_o[1] + rtb_Integrator_i[1];
    rtb_UnitConversion_i = 0.0 * rtb_Integrator_o[2] + rtb_Integrator_i[2];

    /* Product: '<S91>/Body to Stab' */
    for (i = 0; i < 3; i++) {
      frac_0[i] = rtb_VectorConcatenate[i + 6] * rtb_UnitConversion_i +
        (rtb_VectorConcatenate[i + 3] * rtb_UnitConversion +
         rtb_VectorConcatenate[i] * rtb_thrDiff);
    }

    /* End of Product: '<S91>/Body to Stab' */

    /* Gain: '<S159>/Reference Span' incorporates:
     *  Product: '<S159>/Product'
     */
    rtb_Product_ee_idx_0 = rtb_Fcn_m * frac_0[0] * 0.95;
    rtb_Product_ee_idx_1 = rtb_Fcn1_f * frac_0[0] * 0.95;
    rtb_Product_ee_idx_2 = rtb_thetadot * frac_0[0] * 0.95;
    rtb_Product_ee_idx_3 = rtb_kxj * frac_0[0] * 0.95;
    rtb_Product_ee_idx_4 = rtb_ixk * frac_0[0] * 0.95;
    rtb_Product_ee_idx_5 = rtb_jxi * frac_0[0] * 0.95;

    /* Interpolation_n-D generated from: '<S171>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S171>/Constant'
     *  Constant: '<S171>/Constant3'
     */
    frac_30[0] = rtb_InterpolationUsingPrelookup;
    frac_30[1] = 0.0;
    frac_30[2] = rtb_LowAltitudeScaleLength;
    bpIndex_30[0] = rtb_k_b;
    bpIndex_30[1] = 0U;
    bpIndex_30[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_30, frac_30, Simulator_ConstP.pooled78,
                          Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S171>/Interpolation Using Prelookup' */
    frac_31[0] = rtb_InterpolationUsingPrelookup;
    frac_31[1] = rtb_f_n;
    frac_31[2] = rtb_LowAltitudeScaleLength;
    bpIndex_31[0] = rtb_k_b;
    bpIndex_31[1] = rtb_k_d;
    bpIndex_31[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_31, frac_31, Simulator_ConstP.pooled78,
      Simulator_ConstP.pooled91);

    /* Sum: '<S171>/Add' incorporates:
     *  Sum: '<S171>/Subtract'
     */
    rtb_Fcn_m += rtb_Fcn1_f - rtb_Fcn_m;

    /* Interpolation_n-D generated from: '<S172>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S172>/Constant'
     *  Constant: '<S172>/Constant3'
     */
    frac_32[0] = rtb_InterpolationUsingPrelookup;
    frac_32[1] = 0.0;
    frac_32[2] = rtb_LowAltitudeScaleLength;
    bpIndex_32[0] = rtb_k_b;
    bpIndex_32[1] = 0U;
    bpIndex_32[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_32, frac_32, Simulator_ConstP.pooled79,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S172>/Interpolation Using Prelookup' */
    frac_33[0] = rtb_InterpolationUsingPrelookup;
    frac_33[1] = rtb_f_n;
    frac_33[2] = rtb_LowAltitudeScaleLength;
    bpIndex_33[0] = rtb_k_b;
    bpIndex_33[1] = rtb_k_d;
    bpIndex_33[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_33, frac_33, Simulator_ConstP.pooled79,
      Simulator_ConstP.pooled91);

    /* Sum: '<S172>/Add' incorporates:
     *  Product: '<S172>/Product'
     *  Sum: '<S172>/Subtract'
     */
    rtb_Fcn1_f += (rtb_thetadot - rtb_Fcn1_f) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S170>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S170>/Constant'
     *  Constant: '<S170>/Constant3'
     */
    frac_34[0] = rtb_InterpolationUsingPrelookup;
    frac_34[1] = 0.0;
    frac_34[2] = rtb_LowAltitudeScaleLength;
    bpIndex_34[0] = rtb_k_b;
    bpIndex_34[1] = 0U;
    bpIndex_34[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_34, frac_34, Simulator_ConstP.pooled80,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S170>/Interpolation Using Prelookup' */
    frac_35[0] = rtb_InterpolationUsingPrelookup;
    frac_35[1] = rtb_f_n;
    frac_35[2] = rtb_LowAltitudeScaleLength;
    bpIndex_35[0] = rtb_k_b;
    bpIndex_35[1] = rtb_k_d;
    bpIndex_35[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_35, frac_35, Simulator_ConstP.pooled80,
                        Simulator_ConstP.pooled91);

    /* Gain: '<S169>/CL to Cz' incorporates:
     *  Sum: '<S170>/Add'
     *  Sum: '<S170>/Subtract'
     */
    rtb_thetadot = -((rtb_kxj - rtb_thetadot) + rtb_thetadot);

    /* Interpolation_n-D generated from: '<S173>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S173>/Constant'
     *  Constant: '<S173>/Constant3'
     */
    frac_36[0] = rtb_InterpolationUsingPrelookup;
    frac_36[1] = 0.0;
    frac_36[2] = rtb_LowAltitudeScaleLength;
    bpIndex_36[0] = rtb_k_b;
    bpIndex_36[1] = 0U;
    bpIndex_36[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_36, frac_36, Simulator_ConstP.pooled81,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S173>/Interpolation Using Prelookup' */
    frac_37[0] = rtb_InterpolationUsingPrelookup;
    frac_37[1] = rtb_f_n;
    frac_37[2] = rtb_LowAltitudeScaleLength;
    bpIndex_37[0] = rtb_k_b;
    bpIndex_37[1] = rtb_k_d;
    bpIndex_37[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_37, frac_37, Simulator_ConstP.pooled81,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S173>/Add' incorporates:
     *  Product: '<S173>/Product'
     *  Sum: '<S173>/Subtract'
     */
    rtb_kxj += (rtb_ixk - rtb_kxj) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S174>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S174>/Constant'
     *  Constant: '<S174>/Constant3'
     */
    frac_38[0] = rtb_InterpolationUsingPrelookup;
    frac_38[1] = 0.0;
    frac_38[2] = rtb_LowAltitudeScaleLength;
    bpIndex_38[0] = rtb_k_b;
    bpIndex_38[1] = 0U;
    bpIndex_38[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_38, frac_38, Simulator_ConstP.pooled82,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S174>/Interpolation Using Prelookup' */
    frac_39[0] = rtb_InterpolationUsingPrelookup;
    frac_39[1] = rtb_f_n;
    frac_39[2] = rtb_LowAltitudeScaleLength;
    bpIndex_39[0] = rtb_k_b;
    bpIndex_39[1] = rtb_k_d;
    bpIndex_39[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_39, frac_39, Simulator_ConstP.pooled82,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S174>/Add' incorporates:
     *  Sum: '<S174>/Subtract'
     */
    rtb_ixk += rtb_jxi - rtb_ixk;

    /* Interpolation_n-D generated from: '<S175>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S175>/Constant'
     *  Constant: '<S175>/Constant3'
     */
    frac_3a[0] = rtb_InterpolationUsingPrelookup;
    frac_3a[1] = 0.0;
    frac_3a[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3a[0] = rtb_k_b;
    bpIndex_3a[1] = 0U;
    bpIndex_3a[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_3a, frac_3a, Simulator_ConstP.pooled83,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S175>/Interpolation Using Prelookup' */
    frac_3b[0] = rtb_InterpolationUsingPrelookup;
    frac_3b[1] = rtb_f_n;
    frac_3b[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3b[0] = rtb_k_b;
    bpIndex_3b[1] = rtb_k_d;
    bpIndex_3b[2] = rtb_k;
    rtb_pgw_p_idx_0 = intrp3d_l(bpIndex_3b, frac_3b, Simulator_ConstP.pooled83,
      Simulator_ConstP.pooled91);

    /* Sum: '<S175>/Add' incorporates:
     *  Product: '<S175>/Product'
     *  Sum: '<S175>/Subtract'
     */
    rtb_jxi += (rtb_pgw_p_idx_0 - rtb_jxi) * rtb_Sideslip;

    /* Gain: '<S160>/Reference Length' incorporates:
     *  Product: '<S160>/Product'
     */
    rtb_elev = rtb_Fcn_m * frac_0[1] * 0.146;
    rtb_rud = rtb_Fcn1_f * frac_0[1] * 0.146;
    rtb_flaps = rtb_thetadot * frac_0[1] * 0.146;
    rtb_ailOut = rtb_kxj * frac_0[1] * 0.146;
    rtb_Product_d2_idx_4 = rtb_ixk * frac_0[1] * 0.146;
    rtb_Product_d2_idx_5 = rtb_jxi * frac_0[1] * 0.146;

    /* Interpolation_n-D generated from: '<S178>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S178>/Constant'
     *  Constant: '<S178>/Constant3'
     */
    frac_3c[0] = rtb_InterpolationUsingPrelookup;
    frac_3c[1] = 0.0;
    frac_3c[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3c[0] = rtb_k_b;
    bpIndex_3c[1] = 0U;
    bpIndex_3c[2] = rtb_k;
    rtb_Fcn_m = intrp3d_l(bpIndex_3c, frac_3c, Simulator_ConstP.pooled84,
                          Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S178>/Interpolation Using Prelookup' */
    frac_3d[0] = rtb_InterpolationUsingPrelookup;
    frac_3d[1] = rtb_f_n;
    frac_3d[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3d[0] = rtb_k_b;
    bpIndex_3d[1] = rtb_k_d;
    bpIndex_3d[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_3d, frac_3d, Simulator_ConstP.pooled84,
      Simulator_ConstP.pooled91);

    /* Sum: '<S178>/Add' incorporates:
     *  Product: '<S178>/Product'
     *  Sum: '<S178>/Subtract'
     */
    rtb_Fcn_m += (rtb_Fcn1_f - rtb_Fcn_m) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S179>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S179>/Constant'
     *  Constant: '<S179>/Constant3'
     */
    frac_3e[0] = rtb_InterpolationUsingPrelookup;
    frac_3e[1] = 0.0;
    frac_3e[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3e[0] = rtb_k_b;
    bpIndex_3e[1] = 0U;
    bpIndex_3e[2] = rtb_k;
    rtb_Fcn1_f = intrp3d_l(bpIndex_3e, frac_3e, Simulator_ConstP.pooled85,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S179>/Interpolation Using Prelookup' */
    frac_3f[0] = rtb_InterpolationUsingPrelookup;
    frac_3f[1] = rtb_f_n;
    frac_3f[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3f[0] = rtb_k_b;
    bpIndex_3f[1] = rtb_k_d;
    bpIndex_3f[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_3f, frac_3f, Simulator_ConstP.pooled85,
      Simulator_ConstP.pooled91);

    /* Sum: '<S179>/Add' incorporates:
     *  Sum: '<S179>/Subtract'
     */
    rtb_Fcn1_f += rtb_thetadot - rtb_Fcn1_f;

    /* Interpolation_n-D generated from: '<S177>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S177>/Constant'
     *  Constant: '<S177>/Constant3'
     */
    frac_3g[0] = rtb_InterpolationUsingPrelookup;
    frac_3g[1] = 0.0;
    frac_3g[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3g[0] = rtb_k_b;
    bpIndex_3g[1] = 0U;
    bpIndex_3g[2] = rtb_k;
    rtb_thetadot = intrp3d_l(bpIndex_3g, frac_3g, Simulator_ConstP.pooled86,
      Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S177>/Interpolation Using Prelookup' */
    frac_3h[0] = rtb_InterpolationUsingPrelookup;
    frac_3h[1] = rtb_f_n;
    frac_3h[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3h[0] = rtb_k_b;
    bpIndex_3h[1] = rtb_k_d;
    bpIndex_3h[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_3h, frac_3h, Simulator_ConstP.pooled86,
                        Simulator_ConstP.pooled91);

    /* Gain: '<S176>/CL to Cz' incorporates:
     *  Product: '<S177>/Product'
     *  Sum: '<S177>/Add'
     *  Sum: '<S177>/Subtract'
     */
    rtb_thetadot = -((rtb_kxj - rtb_thetadot) * rtb_Sideslip + rtb_thetadot);

    /* Interpolation_n-D generated from: '<S180>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S180>/Constant'
     *  Constant: '<S180>/Constant3'
     */
    frac_3i[0] = rtb_InterpolationUsingPrelookup;
    frac_3i[1] = 0.0;
    frac_3i[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3i[0] = rtb_k_b;
    bpIndex_3i[1] = 0U;
    bpIndex_3i[2] = rtb_k;
    rtb_kxj = intrp3d_l(bpIndex_3i, frac_3i, Simulator_ConstP.pooled87,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S180>/Interpolation Using Prelookup' */
    frac_3j[0] = rtb_InterpolationUsingPrelookup;
    frac_3j[1] = rtb_f_n;
    frac_3j[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3j[0] = rtb_k_b;
    bpIndex_3j[1] = rtb_k_d;
    bpIndex_3j[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_3j, frac_3j, Simulator_ConstP.pooled87,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S180>/Add' incorporates:
     *  Sum: '<S180>/Subtract'
     */
    rtb_kxj += rtb_ixk - rtb_kxj;

    /* Interpolation_n-D generated from: '<S181>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S181>/Constant'
     *  Constant: '<S181>/Constant3'
     */
    frac_3k[0] = rtb_InterpolationUsingPrelookup;
    frac_3k[1] = 0.0;
    frac_3k[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3k[0] = rtb_k_b;
    bpIndex_3k[1] = 0U;
    bpIndex_3k[2] = rtb_k;
    rtb_ixk = intrp3d_l(bpIndex_3k, frac_3k, Simulator_ConstP.pooled88,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S181>/Interpolation Using Prelookup' */
    frac_3l[0] = rtb_InterpolationUsingPrelookup;
    frac_3l[1] = rtb_f_n;
    frac_3l[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3l[0] = rtb_k_b;
    bpIndex_3l[1] = rtb_k_d;
    bpIndex_3l[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_3l, frac_3l, Simulator_ConstP.pooled88,
                        Simulator_ConstP.pooled91);

    /* Sum: '<S181>/Add' incorporates:
     *  Product: '<S181>/Product'
     *  Sum: '<S181>/Subtract'
     */
    rtb_ixk += (rtb_jxi - rtb_ixk) * rtb_Sideslip;

    /* Interpolation_n-D generated from: '<S182>/Interpolation Using Prelookup1' incorporates:
     *  Constant: '<S182>/Constant'
     *  Constant: '<S182>/Constant3'
     */
    frac_3m[0] = rtb_InterpolationUsingPrelookup;
    frac_3m[1] = 0.0;
    frac_3m[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3m[0] = rtb_k_b;
    bpIndex_3m[1] = 0U;
    bpIndex_3m[2] = rtb_k;
    rtb_jxi = intrp3d_l(bpIndex_3m, frac_3m, Simulator_ConstP.pooled89,
                        Simulator_ConstP.pooled91);

    /* Interpolation_n-D generated from: '<S182>/Interpolation Using Prelookup' */
    frac_3n[0] = rtb_InterpolationUsingPrelookup;
    frac_3n[1] = rtb_f_n;
    frac_3n[2] = rtb_LowAltitudeScaleLength;
    bpIndex_3n[0] = rtb_k_b;
    bpIndex_3n[1] = rtb_k_d;
    bpIndex_3n[2] = rtb_k;
    rtb_InterpolationUsingPrelookup = intrp3d_l(bpIndex_3n, frac_3n,
      Simulator_ConstP.pooled89, Simulator_ConstP.pooled91);

    /* Sum: '<S182>/Add' incorporates:
     *  Sum: '<S182>/Subtract'
     */
    rtb_jxi += rtb_InterpolationUsingPrelookup - rtb_jxi;

    /* Saturate: '<S91>/Saturation' */
    if (rtb_Saturation_m <= 0.1) {
      rtb_Saturation_m = 0.1;
    }

    /* End of Saturate: '<S91>/Saturation' */

    /* Sum: '<S70>/Sum' incorporates:
     *  Gain: '<S161>/Reference Span'
     *  Product: '<S161>/Product'
     *  Product: '<S91>/Divide'
     *  Sum: '<S91>/Sum1'
     */
    rtb_Sum_a_0[0] = (rtb_Fcn_m * frac_0[2] * 0.95 + (rtb_Product_ee_idx_0 +
      rtb_elev)) / rtb_Saturation_m + (rtb_sigma_ugsigma_vg + rtb_Product_cl[0]);
    rtb_Sum_a_0[1] = (rtb_Fcn1_f * frac_0[2] * 0.95 + (rtb_Product_ee_idx_1 +
      rtb_rud)) / rtb_Saturation_m + (rtb_Add_bm + rtb_Product_cl[1]);
    rtb_Sum_a_0[2] = (rtb_thetadot * frac_0[2] * 0.95 + (rtb_Product_ee_idx_2 +
      rtb_flaps)) / rtb_Saturation_m + (rtb_CLtoCz + rtb_Product_cl[2]);
    rtb_Sum_a_0[3] = (rtb_kxj * frac_0[2] * 0.95 + (rtb_Product_ee_idx_3 +
      rtb_ailOut)) / rtb_Saturation_m + (rtb_Add_kh + rtb_Product_cl[3]);
    rtb_Sum_a_0[4] = (rtb_ixk * frac_0[2] * 0.95 + (rtb_Product_ee_idx_4 +
      rtb_Product_d2_idx_4)) / rtb_Saturation_m + (rtb_Add_bc + rtb_Product_cl[4]);
    rtb_Sum_a_0[5] = (rtb_jxi * frac_0[2] * 0.95 + (rtb_Product_ee_idx_5 +
      rtb_Product_d2_idx_5)) / rtb_Saturation_m + (rtb_Add_hq + rtb_Product_cl[5]);
    for (i = 0; i < 6; i++) {
      rtb_Product_cl[i] = rtb_Sum_a_0[i];
    }

    /* End of Sum: '<S70>/Sum' */
    for (i = 0; i < 3; i++) {
      /* Product: '<S197>/Product' incorporates:
       *  Product: '<S93>/F stab to body'
       */
      rtb_Integrator_o[i] = rtb_Fcn1 * (rtb_DCM_bs[i + 6] * rtb_Product_cl[2] +
        (rtb_DCM_bs[i + 3] * rtb_Product_cl[1] + rtb_DCM_bs[i] * rtb_Product_cl
         [0]));
    }

    /* GravityWGS84: '<S245>/WGS84 Gravity Model  ' */
    rtb_LowAltitudeScaleLength = Simulator_B.Add[0] * 0.017453292519943295;
    rtb_UnitConversion = std::abs(rtb_LowAltitudeScaleLength);
    rtb_Saturation_m = 1.0;
    if (rtb_UnitConversion > 3.1415926535897931) {
      if (rtb_LowAltitudeScaleLength < -3.1415926535897931) {
        rtb_Saturation_m = -1.0;
      }

      if (rtIsInf(rtb_UnitConversion + 3.1415926535897931)) {
        rtb_UnitConversion_i = (rtNaN);
      } else {
        rtb_UnitConversion_i = std::fmod(rtb_UnitConversion + 3.1415926535897931,
          6.2831853071795862);
        rEQ0 = (rtb_UnitConversion_i == 0.0);
        if (!rEQ0) {
          rtb_InterpolationUsingPrelookup = (rtb_UnitConversion +
            3.1415926535897931) / 6.2831853071795862;
          rEQ0 = (std::abs(rtb_InterpolationUsingPrelookup - std::floor
                           (rtb_InterpolationUsingPrelookup + 0.5)) <=
                  2.2204460492503131E-16 * rtb_InterpolationUsingPrelookup);
        }

        if (rEQ0) {
          rtb_UnitConversion_i = 0.0;
        }
      }

      rtb_LowAltitudeScaleLength = (rtb_UnitConversion_i - 3.1415926535897931) *
        rtb_Saturation_m;
      rtb_UnitConversion = std::abs(rtb_LowAltitudeScaleLength);
    }

    if (rtb_UnitConversion > 1.5707963267948966) {
      if (rtb_LowAltitudeScaleLength > 1.5707963267948966) {
        rtb_LowAltitudeScaleLength = 1.5707963267948966 - (rtb_UnitConversion -
          1.5707963267948966);
      }

      if (rtb_LowAltitudeScaleLength < -1.5707963267948966) {
        rtb_LowAltitudeScaleLength = -(1.5707963267948966 - (rtb_UnitConversion
          - 1.5707963267948966));
      }
    }

    rtb_Add_kh = std::sin(rtb_LowAltitudeScaleLength);
    rtb_Saturation_m = rtb_Add_kh * rtb_Add_kh;

    /* Gain: '<S72>/Weight in Earth Axes' incorporates:
     *  GravityWGS84: '<S245>/WGS84 Gravity Model  '
     */
    rtb_thrDiff = ((1.0 - (1.006802597171564 - 2.0 * rtb_Saturation_m /
      298.257223563) * 2.0 * Simulator_B.Add[2] / 6.378137E+6) + 3.0 *
                   Simulator_B.Add[2] * Simulator_B.Add[2] / 4.0680631590769E+13)
      * ((0.00193185265241 * rtb_Saturation_m + 1.0) * 9.7803253359 / std::sqrt
         (1.0 - 0.00669437999014 * rtb_Saturation_m)) * 2.6;

    /* Product: '<S72>/Inertial to Body' */
    for (i = 0; i < 3; i++) {
      rtb_Sum2_m[i] = Simulator_B.VectorConcatenate[i + 6] * rtb_thrDiff +
        (Simulator_B.VectorConcatenate[i + 3] * 0.0 +
         Simulator_B.VectorConcatenate[i] * 0.0);
    }

    /* Sum: '<S6>/Sum' incorporates:
     *  Constant: '<S6>/Constant'
     *  Product: '<S58>/i x j'
     *  Product: '<S58>/j x k'
     *  Product: '<S58>/k x i'
     *  Product: '<S59>/i x k'
     *  Product: '<S59>/j x i'
     *  Product: '<S59>/k x j'
     *  Product: '<S6>/Product'
     *  Product: '<S72>/Inertial to Body'
     *  Sum: '<S15>/Sum'
     *  Sum: '<S7>/Sum'
     */
    frac_0[0] = (rtb_Integrator_o[0] + rtb_Sum2_m[0]) / 2.6 + (rtb_Mstabtobody[1]
      * rtb_Integrator_i[2] - rtb_Mstabtobody[2] * rtb_Integrator_i[1]);
    frac_0[1] = (rtb_Integrator_o[1] + rtb_Sum2_m[1]) / 2.6 + (rtb_Mstabtobody[2]
      * rtb_Integrator_i[0] - rtb_Mstabtobody[0] * rtb_Integrator_i[2]);
    frac_0[2] = (rtb_Integrator_o[2] + rtb_Sum2_m[2]) / 2.6 + (rtb_Mstabtobody[0]
      * rtb_Integrator_i[1] - rtb_Mstabtobody[1] * rtb_Integrator_i[0]);

    /* Sum: '<S210>/Add' */
    rtb_thrDiff = (Simulator_B.VectorConcatenate[0] +
                   Simulator_B.VectorConcatenate[4]) +
      Simulator_B.VectorConcatenate[8];

    /* If: '<S206>/If' */
    if (rtmIsMajorTimeStep((&Simulator_M))) {
      rtAction = static_cast<int8_T>(!(rtb_thrDiff > 0.0));
      Simulator_DW.If_ActiveSubsystem = rtAction;
    } else {
      rtAction = Simulator_DW.If_ActiveSubsystem;
    }

    switch (rtAction) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S206>/Positive Trace' incorporates:
       *  ActionPort: '<S208>/Action Port'
       */
      /* Sqrt: '<S208>/sqrt' incorporates:
       *  Constant: '<S208>/Constant'
       *  Sum: '<S208>/Sum'
       */
      rtb_thrDiff = std::sqrt(rtb_thrDiff + 1.0);

      /* Gain: '<S208>/Gain' */
      Simulator_B.Merge[0] = 0.5 * rtb_thrDiff;

      /* Gain: '<S208>/Gain1' */
      rtb_thrDiff *= 2.0;

      /* Product: '<S208>/Product' incorporates:
       *  Sum: '<S230>/Add'
       *  Sum: '<S231>/Add'
       *  Sum: '<S232>/Add'
       */
      Simulator_B.Merge[1] = (Simulator_B.VectorConcatenate[7] -
        Simulator_B.VectorConcatenate[5]) / rtb_thrDiff;
      Simulator_B.Merge[2] = (Simulator_B.VectorConcatenate[2] -
        Simulator_B.VectorConcatenate[6]) / rtb_thrDiff;
      Simulator_B.Merge[3] = (Simulator_B.VectorConcatenate[3] -
        Simulator_B.VectorConcatenate[1]) / rtb_thrDiff;

      /* End of Outputs for SubSystem: '<S206>/Positive Trace' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S206>/Negative Trace' incorporates:
       *  ActionPort: '<S207>/Action Port'
       */
      /* If: '<S207>/Find Maximum Diagonal Value' */
      if (rtmIsMajorTimeStep((&Simulator_M))) {
        if ((Simulator_B.VectorConcatenate[4] > Simulator_B.VectorConcatenate[0])
            && (Simulator_B.VectorConcatenate[4] >
                Simulator_B.VectorConcatenate[8])) {
          rtAction = 0;
        } else if (Simulator_B.VectorConcatenate[8] >
                   Simulator_B.VectorConcatenate[0]) {
          rtAction = 1;
        } else {
          rtAction = 2;
        }

        Simulator_DW.FindMaximumDiagonalValue_Active = rtAction;
      } else {
        rtAction = Simulator_DW.FindMaximumDiagonalValue_Active;
      }

      switch (rtAction) {
       case 0:
        /* Outputs for IfAction SubSystem: '<S207>/Maximum Value at DCM(2,2)' incorporates:
         *  ActionPort: '<S212>/Action Port'
         */
        /* Sqrt: '<S212>/sqrt' incorporates:
         *  Constant: '<S224>/Constant'
         *  Sum: '<S224>/Add'
         */
        rtb_thrDiff = std::sqrt(((Simulator_B.VectorConcatenate[4] -
          Simulator_B.VectorConcatenate[0]) - Simulator_B.VectorConcatenate[8])
          + 1.0);

        /* Gain: '<S212>/Gain' */
        Simulator_B.Merge[2] = 0.5 * rtb_thrDiff;

        /* Switch: '<S223>/Switch' incorporates:
         *  Constant: '<S223>/Constant1'
         */
        if (rtb_thrDiff != 0.0) {
          frac[0] = 0.5;
          frac[1] = rtb_thrDiff;
        } else {
          frac[0] = 0.0;
          frac[1] = 1.0;
        }

        /* End of Switch: '<S223>/Switch' */

        /* Product: '<S223>/Product' */
        rtb_thrDiff = frac[0] / frac[1];

        /* Gain: '<S212>/Gain1' incorporates:
         *  Product: '<S212>/Product'
         *  Sum: '<S222>/Add'
         */
        Simulator_B.Merge[1] = (Simulator_B.VectorConcatenate[1] +
          Simulator_B.VectorConcatenate[3]) * rtb_thrDiff;

        /* Gain: '<S212>/Gain3' incorporates:
         *  Product: '<S212>/Product'
         *  Sum: '<S221>/Add'
         */
        Simulator_B.Merge[3] = (Simulator_B.VectorConcatenate[5] +
          Simulator_B.VectorConcatenate[7]) * rtb_thrDiff;

        /* Gain: '<S212>/Gain4' incorporates:
         *  Product: '<S212>/Product'
         *  Sum: '<S220>/Add'
         */
        Simulator_B.Merge[0] = (Simulator_B.VectorConcatenate[2] -
          Simulator_B.VectorConcatenate[6]) * rtb_thrDiff;

        /* End of Outputs for SubSystem: '<S207>/Maximum Value at DCM(2,2)' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S207>/Maximum Value at DCM(3,3)' incorporates:
         *  ActionPort: '<S213>/Action Port'
         */
        /* Sqrt: '<S213>/sqrt' incorporates:
         *  Constant: '<S229>/Constant'
         *  Sum: '<S229>/Add'
         */
        rtb_thrDiff = std::sqrt(((Simulator_B.VectorConcatenate[8] -
          Simulator_B.VectorConcatenate[0]) - Simulator_B.VectorConcatenate[4])
          + 1.0);

        /* Gain: '<S213>/Gain' */
        Simulator_B.Merge[3] = 0.5 * rtb_thrDiff;

        /* Switch: '<S228>/Switch' incorporates:
         *  Constant: '<S228>/Constant1'
         */
        if (rtb_thrDiff != 0.0) {
          frac[0] = 0.5;
          frac[1] = rtb_thrDiff;
        } else {
          frac[0] = 0.0;
          frac[1] = 1.0;
        }

        /* End of Switch: '<S228>/Switch' */

        /* Product: '<S228>/Product' */
        rtb_thrDiff = frac[0] / frac[1];

        /* Gain: '<S213>/Gain1' incorporates:
         *  Product: '<S213>/Product'
         *  Sum: '<S225>/Add'
         */
        Simulator_B.Merge[1] = (Simulator_B.VectorConcatenate[2] +
          Simulator_B.VectorConcatenate[6]) * rtb_thrDiff;

        /* Gain: '<S213>/Gain2' incorporates:
         *  Product: '<S213>/Product'
         *  Sum: '<S226>/Add'
         */
        Simulator_B.Merge[2] = (Simulator_B.VectorConcatenate[5] +
          Simulator_B.VectorConcatenate[7]) * rtb_thrDiff;

        /* Gain: '<S213>/Gain3' incorporates:
         *  Product: '<S213>/Product'
         *  Sum: '<S227>/Add'
         */
        Simulator_B.Merge[0] = (Simulator_B.VectorConcatenate[3] -
          Simulator_B.VectorConcatenate[1]) * rtb_thrDiff;

        /* End of Outputs for SubSystem: '<S207>/Maximum Value at DCM(3,3)' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S207>/Maximum Value at DCM(1,1)' incorporates:
         *  ActionPort: '<S211>/Action Port'
         */
        /* Sqrt: '<S211>/sqrt' incorporates:
         *  Constant: '<S219>/Constant'
         *  Sum: '<S219>/Add'
         */
        rtb_thrDiff = std::sqrt(((Simulator_B.VectorConcatenate[0] -
          Simulator_B.VectorConcatenate[4]) - Simulator_B.VectorConcatenate[8])
          + 1.0);

        /* Gain: '<S211>/Gain' */
        Simulator_B.Merge[1] = 0.5 * rtb_thrDiff;

        /* Switch: '<S218>/Switch' incorporates:
         *  Constant: '<S218>/Constant1'
         */
        if (rtb_thrDiff != 0.0) {
          frac[0] = 0.5;
          frac[1] = rtb_thrDiff;
        } else {
          frac[0] = 0.0;
          frac[1] = 1.0;
        }

        /* End of Switch: '<S218>/Switch' */

        /* Product: '<S218>/Product' */
        rtb_thrDiff = frac[0] / frac[1];

        /* Gain: '<S211>/Gain1' incorporates:
         *  Product: '<S211>/Product'
         *  Sum: '<S217>/Add'
         */
        Simulator_B.Merge[2] = (Simulator_B.VectorConcatenate[1] +
          Simulator_B.VectorConcatenate[3]) * rtb_thrDiff;

        /* Gain: '<S211>/Gain2' incorporates:
         *  Product: '<S211>/Product'
         *  Sum: '<S215>/Add'
         */
        Simulator_B.Merge[3] = (Simulator_B.VectorConcatenate[2] +
          Simulator_B.VectorConcatenate[6]) * rtb_thrDiff;

        /* Gain: '<S211>/Gain3' incorporates:
         *  Product: '<S211>/Product'
         *  Sum: '<S216>/Add'
         */
        Simulator_B.Merge[0] = (Simulator_B.VectorConcatenate[7] -
          Simulator_B.VectorConcatenate[5]) * rtb_thrDiff;

        /* End of Outputs for SubSystem: '<S207>/Maximum Value at DCM(1,1)' */
        break;
      }

      /* End of If: '<S207>/Find Maximum Diagonal Value' */
      /* End of Outputs for SubSystem: '<S206>/Negative Trace' */
      break;
    }

    /* End of If: '<S206>/If' */

    /* DataStoreWrite: '<S205>/Data Store Write' incorporates:
     *  BusCreator: '<S205>/Bus Creator1'
     *  Integrator: '<S60>/Integrator'
     *  Product: '<S6>/MatrixMultiply1'
     */
    Simulator_DW.ACBus_o.time = rtb_time;
    for (i = 0; i < 3; i++) {
      Simulator_DW.ACBus_o.Wb[i] = rtb_Integrator_i[i];
      Simulator_DW.ACBus_o.Ve[i] = rtb_Product_ly[i];

      /* Sum: '<S6>/Add' incorporates:
       *  Constant: '<S6>/Constant3'
       *  Math: '<S6>/Transpose'
       *  Product: '<S6>/MatrixMultiply'
       *  Product: '<S6>/Product1'
       */
      rtb_Sum2_m[i] = static_cast<real_T>(rtb_LogicalOperator2_p) *
        (rtb_Transpose_tmp[i + 6] * frac_0[2] + (rtb_Transpose_tmp[i + 3] *
          frac_0[1] + rtb_Transpose_tmp[i] * frac_0[0])) +
        Simulator_ConstP.Constant3_Value[i];
    }

    for (i = 0; i < 3; i++) {
      Simulator_DW.ACBus_o.Ab[i] = 0.0;
      Simulator_DW.ACBus_o.Ab[i] += Simulator_B.VectorConcatenate[i] *
        rtb_Sum2_m[0];
      Simulator_DW.ACBus_o.Ab[i] += Simulator_B.VectorConcatenate[i + 3] *
        rtb_Sum2_m[1];
      Simulator_DW.ACBus_o.Ab[i] += Simulator_B.VectorConcatenate[i + 6] *
        rtb_Sum2_m[2];
    }

    Simulator_DW.ACBus_o.quat[0] = Simulator_B.Merge[0];
    Simulator_DW.ACBus_o.quat[1] = Simulator_B.Merge[1];
    Simulator_DW.ACBus_o.quat[2] = Simulator_B.Merge[2];
    Simulator_DW.ACBus_o.quat[3] = Simulator_B.Merge[3];
    Simulator_DW.ACBus_o.Xe[0] = Simulator_X.Integrator_CSTATE_a[0];
    Simulator_DW.ACBus_o.Xe[1] = Simulator_X.Integrator_CSTATE_a[1];
    Simulator_DW.ACBus_o.Xe[2] = Simulator_X.Integrator_CSTATE_a[2];

    /* End of DataStoreWrite: '<S205>/Data Store Write' */

    /* Clock: '<S24>/Clock' incorporates:
     *  Clock: '<S41>/Clock'
     *  Clock: '<S49>/Clock'
     *  Clock: '<S64>/Clock'
     */
    rtb_time = (&Simulator_M)->Timing.t[0];
    Simulator_B.Clock = rtb_time;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* If: '<S209>/If1' */
      if (rtmIsMajorTimeStep((&Simulator_M))) {
        Simulator_DW.If1_ActiveSubsystem = -1;
      }

      /* End of If: '<S209>/If1' */

      /* Outputs for Triggered SubSystem: '<S24>/Triggered Subsystem' */
      Simulator_TriggeredSubsystem(Simulator_B.LogicalOperator1_k,
        &Simulator_PrevZCX.TriggeredSubsystem);

      /* End of Outputs for SubSystem: '<S24>/Triggered Subsystem' */
    }

    for (i = 0; i < 3; i++) {
      /* Switch: '<S20>/Switch' incorporates:
       *  Constant: '<S20>/Constant1'
       *  Logic: '<S20>/Logical Operator1'
       *  Logic: '<S20>/Logical Operator2'
       *  Logic: '<S20>/Logical Operator3'
       *  RelationalOperator: '<S20>/Relational Operator2'
       *  RelationalOperator: '<S20>/Relational Operator3'
       */
      if (((frac_0[i] >= 0.0) && rtb_LowerRelop1_g[i]) ||
          (rtb_LogicalOperator3_d[i] && (frac_0[i] <= 0.0))) {
        Simulator_B.Switch[i] = 0.0;
      } else {
        Simulator_B.Switch[i] = frac_0[i];
      }

      /* End of Switch: '<S20>/Switch' */

      /* Product: '<S29>/Product' */
      frac_0[i] = Simulator_ConstB.Selector[i + 6] * rtb_Integrator_i[2] +
        (Simulator_ConstB.Selector[i + 3] * rtb_Integrator_i[1] +
         Simulator_ConstB.Selector[i] * rtb_Integrator_i[0]);
    }

    /* Sum: '<S28>/Sum' incorporates:
     *  Product: '<S31>/i x j'
     *  Product: '<S31>/j x k'
     *  Product: '<S31>/k x i'
     *  Product: '<S32>/i x k'
     *  Product: '<S32>/j x i'
     *  Product: '<S32>/k x j'
     */
    rtb_Mstabtobody[0] = rtb_Integrator_i[1] * frac_0[2];
    rtb_Mstabtobody[1] = rtb_Integrator_i[2] * frac_0[0];
    rtb_Mstabtobody[2] = rtb_Integrator_i[0] * frac_0[1];
    rtb_Gain3[0] = rtb_Integrator_i[2] * frac_0[1];
    rtb_Gain3[1] = rtb_Integrator_i[0] * frac_0[2];
    rtb_Gain3[2] = rtb_Integrator_i[1] * frac_0[0];

    /* Sum: '<S198>/Sum' incorporates:
     *  Product: '<S202>/i x j'
     *  Product: '<S202>/j x k'
     *  Product: '<S202>/k x i'
     *  Product: '<S203>/i x k'
     *  Product: '<S203>/j x i'
     *  Product: '<S203>/k x j'
     */
    rtb_Sum2_m[0] = rtb_Integrator_o[1] * Simulator_ConstB.Sum[2];
    rtb_Sum2_m[1] = rtb_Integrator_o[2] * Simulator_ConstB.Sum[0];
    rtb_Sum2_m[2] = rtb_Integrator_o[0] * Simulator_ConstB.Sum[1];
    frac_1[0] = rtb_Integrator_o[2] * Simulator_ConstB.Sum[1];
    frac_1[1] = rtb_Integrator_o[0] * Simulator_ConstB.Sum[2];
    frac_1[2] = rtb_Integrator_o[1] * Simulator_ConstB.Sum[0];

    /* Product: '<S197>/Product1' incorporates:
     *  Constant: '<S197>/Constant'
     *  Constant: '<S197>/Constant1'
     */
    frac_2[0] = 1.9 * rtb_Fcn1;
    frac_2[1] = 0.292 * rtb_Fcn1;
    frac_2[2] = 1.9 * rtb_Fcn1;
    for (i = 0; i < 3; i++) {
      /* Sum: '<S11>/Sum2' incorporates:
       *  Product: '<S197>/Product3'
       *  Product: '<S30>/Product'
       *  Product: '<S93>/M stab to body'
       *  Sum: '<S197>/Sum1'
       *  Sum: '<S198>/Sum'
       *  Sum: '<S28>/Sum'
       */
      frac_0[i] = (((rtb_Sum2_m[i] - frac_1[i]) + (rtb_DCM_bs[i + 6] *
        rtb_Product_cl[5] + (rtb_DCM_bs[i + 3] * rtb_Product_cl[4] +
        rtb_DCM_bs[i] * rtb_Product_cl[3])) * frac_2[i]) - (0.0 *
        rtb_Integrator_i[2] + (0.0 * rtb_Integrator_i[1] + 0.0 *
        rtb_Integrator_i[0]))) - (rtb_Mstabtobody[i] - rtb_Gain3[i]);

      /* Sum: '<S198>/Sum' incorporates:
       *  Trigonometry: '<S34>/sincos'
       */
      rtb_Integrator_o[i] = std::cos(rtb_sincos_o1[i]);

      /* Trigonometry: '<S34>/sincos' */
      rtb_sincos_o1[i] = std::sin(rtb_sincos_o1[i]);
    }

    /* Product: '<S11>/Product2' */
    rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(frac_0, Simulator_ConstB.Selector2,
      rtb_Mstabtobody);

    /* Fcn: '<S34>/phidot' incorporates:
     *  Fcn: '<S34>/psidot'
     */
    rtb_Fcn1 = rtb_Integrator_i[1] * rtb_sincos_o1[0] + rtb_Integrator_i[2] *
      rtb_Integrator_o[0];
    rtb_Fcn_m = rtb_Fcn1 * (rtb_sincos_o1[1] / rtb_Integrator_o[1]) +
      rtb_Integrator_i[0];

    /* Fcn: '<S34>/psidot' */
    rtb_Fcn1_f = rtb_Fcn1 / rtb_Integrator_o[1];

    /* Fcn: '<S34>/thetadot' */
    rtb_thetadot = rtb_Integrator_i[1] * rtb_Integrator_o[0] - rtb_Integrator_i
      [2] * rtb_sincos_o1[0];

    /* Clock: '<S41>/Clock' */
    Simulator_B.Clock_o = rtb_time;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Outputs for Triggered SubSystem: '<S41>/Triggered Subsystem' */
      Simulator_TriggeredSubsystem(Simulator_B.LogicalOperator1_b,
        &Simulator_PrevZCX.TriggeredSubsystem_m);

      /* End of Outputs for SubSystem: '<S41>/Triggered Subsystem' */
    }

    /* Switch: '<S37>/Switch' incorporates:
     *  Constant: '<S37>/Constant1'
     *  Logic: '<S37>/Logical Operator1'
     *  Logic: '<S37>/Logical Operator2'
     *  Logic: '<S37>/Logical Operator3'
     *  RelationalOperator: '<S37>/Relational Operator2'
     *  RelationalOperator: '<S37>/Relational Operator3'
     */
    if (((rtb_Fcn_m >= 0.0) && rtb_LowerRelop1_idx_0) ||
        (rtb_RelationalOperator_p_idx_0 && (rtb_Fcn_m <= 0.0))) {
      Simulator_B.Switch_d[0] = 0.0;
    } else {
      Simulator_B.Switch_d[0] = rtb_Fcn_m;
    }

    if (((rtb_thetadot >= 0.0) && rtb_LowerRelop1_idx_1) ||
        (rtb_RelationalOperator_p_idx_1 && (rtb_thetadot <= 0.0))) {
      Simulator_B.Switch_d[1] = 0.0;
    } else {
      Simulator_B.Switch_d[1] = rtb_thetadot;
    }

    if (((rtb_Fcn1_f >= 0.0) && rtb_LowerRelop1) || (rtb_RelationalOperator_c &&
         (rtb_Fcn1_f <= 0.0))) {
      Simulator_B.Switch_d[2] = 0.0;
    } else {
      Simulator_B.Switch_d[2] = rtb_Fcn1_f;
    }

    /* End of Switch: '<S37>/Switch' */

    /* Clock: '<S49>/Clock' */
    Simulator_B.Clock_i = rtb_time;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Outputs for Triggered SubSystem: '<S49>/Triggered Subsystem' */
      Simulator_TriggeredSubsystem(Simulator_B.LogicalOperator1,
        &Simulator_PrevZCX.TriggeredSubsystem_g);

      /* End of Outputs for SubSystem: '<S49>/Triggered Subsystem' */
    }

    /* Switch: '<S45>/Switch' incorporates:
     *  Constant: '<S45>/Constant1'
     *  Logic: '<S45>/Logical Operator1'
     *  Logic: '<S45>/Logical Operator2'
     *  Logic: '<S45>/Logical Operator3'
     *  RelationalOperator: '<S45>/Relational Operator2'
     *  RelationalOperator: '<S45>/Relational Operator3'
     */
    if (((rtb_Mstabtobody[0] >= 0.0) && rtb_RelationalOperator1_idx_0) ||
        (rtb_RelationalOperator_idx_0 && (rtb_Mstabtobody[0] <= 0.0))) {
      Simulator_B.Switch_i[0] = 0.0;
    } else {
      Simulator_B.Switch_i[0] = rtb_Mstabtobody[0];
    }

    if (((rtb_Mstabtobody[1] >= 0.0) && rtb_RelationalOperator1_idx_1) ||
        (rtb_RelationalOperator_idx_1 && (rtb_Mstabtobody[1] <= 0.0))) {
      Simulator_B.Switch_i[1] = 0.0;
    } else {
      Simulator_B.Switch_i[1] = rtb_Mstabtobody[1];
    }

    if (((rtb_Mstabtobody[2] >= 0.0) && rtb_RelationalOperator1) ||
        (rtb_RelationalOperator && (rtb_Mstabtobody[2] <= 0.0))) {
      Simulator_B.Switch_i[2] = 0.0;
    } else {
      Simulator_B.Switch_i[2] = rtb_Mstabtobody[2];
    }

    /* End of Switch: '<S45>/Switch' */
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Logic: '<S55>/NOT' */
      Simulator_B.NOT_k = !Simulator_B.Memory;

      /* Memory: '<S57>/Memory' */
      Simulator_B.Memory_o = Simulator_DW.Memory_PreviousInput_g;
    }

    /* CombinatorialLogic: '<S57>/Logic' incorporates:
     *  Fcn: '<S55>/Fcn'
     *  Fcn: '<S55>/Fcn1'
     *  Integrator: '<S60>/Integrator'
     *  Logic: '<S55>/Logical Operator2'
     *  Logic: '<S55>/Logical Operator3'
     */
    rtb_k_b = (((static_cast<uint32_T>((Simulator_B.NOT_k &&
      (-Simulator_X.Integrator_CSTATE_a[2] < 0.0))) << 1) + (Simulator_B.Memory &&
      (-Simulator_X.Integrator_CSTATE_a[2] > 0.5))) << 1) + Simulator_B.Memory_o;
    Simulator_B.Logic[0U] = Simulator_ConstP.Logic_table[rtb_k_b];
    Simulator_B.Logic[1U] = Simulator_ConstP.Logic_table[rtb_k_b + 8U];

    /* Clock: '<S64>/Clock' */
    Simulator_B.Clock_c = rtb_time;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Outputs for Triggered SubSystem: '<S64>/Triggered Subsystem' */
      Simulator_TriggeredSubsystem(Simulator_B.LogicalOperator1_j,
        &Simulator_PrevZCX.TriggeredSubsystem_h);

      /* End of Outputs for SubSystem: '<S64>/Triggered Subsystem' */
    }

    /* Switch: '<S60>/Switch' incorporates:
     *  Constant: '<S60>/Constant1'
     *  Logic: '<S60>/Logical Operator1'
     *  Logic: '<S60>/Logical Operator2'
     *  Logic: '<S60>/Logical Operator3'
     *  RelationalOperator: '<S60>/Relational Operator2'
     *  RelationalOperator: '<S60>/Relational Operator3'
     */
    if (((rtb_Product_ly[0] >= 0.0) && rtb_LowerRelop1_j_idx_0) ||
        (rtb_RelationalOperator_e_idx_0 && (rtb_Product_ly[0] <= 0.0))) {
      Simulator_B.Switch_j[0] = 0.0;
    } else {
      Simulator_B.Switch_j[0] = rtb_Product_ly[0];
    }

    if (((rtb_Product_ly[1] >= 0.0) && rtb_LowerRelop1_j_idx_1) ||
        (rtb_RelationalOperator_e_idx_1 && (rtb_Product_ly[1] <= 0.0))) {
      Simulator_B.Switch_j[1] = 0.0;
    } else {
      Simulator_B.Switch_j[1] = rtb_Product_ly[1];
    }

    if (((rtb_Product_ly[2] >= 0.0) && rtb_LowerRelop1_j) ||
        (rtb_RelationalOperator_l && (rtb_Product_ly[2] <= 0.0))) {
      Simulator_B.Switch_j[2] = 0.0;
    } else {
      Simulator_B.Switch_j[2] = rtb_Product_ly[2];
    }

    /* End of Switch: '<S60>/Switch' */
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Logic: '<S8>/Logical Operator' incorporates:
       *  Fcn: '<S8>/Fcn'
       */
      rtb_LogicalOperator = (rtb_platformStart || (rtb_thr > 0.1));
    }
  }

  if (rtmIsMajorTimeStep((&Simulator_M))) {
    int32_T i;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Update for Memory: '<S55>/Memory' */
      Simulator_DW.Memory_PreviousInput = Simulator_B.Logic[0];

      /* Update for Memory: '<S8>/Memory' */
      Simulator_DW.Memory_PreviousInput_f = rtb_LogicalOperator;

      /* Update for Memory: '<S48>/Memory' */
      Simulator_DW.Memory_PreviousInput_l[0] = Simulator_B.LogicalOperator[0];
      Simulator_DW.Memory_PreviousInput_l[1] = Simulator_B.LogicalOperator[1];
      Simulator_DW.Memory_PreviousInput_l[2] = Simulator_B.LogicalOperator[2];

      /* Update for Memory: '<S40>/Memory' */
      Simulator_DW.Memory_PreviousInput_a[0] = Simulator_B.LogicalOperator_c[0];
      Simulator_DW.Memory_PreviousInput_a[1] = Simulator_B.LogicalOperator_c[1];
      Simulator_DW.Memory_PreviousInput_a[2] = Simulator_B.LogicalOperator_c[2];
    }

    /* Update for Integrator: '<S45>/Integrator' */
    Simulator_DW.Integrator_IWORK = 0;

    /* Update for Integrator: '<S37>/Integrator' */
    Simulator_DW.Integrator_IWORK_o = 0;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Update for Memory: '<S23>/Memory' */
      Simulator_DW.Memory_PreviousInput_a5[0] = Simulator_B.LogicalOperator_b[0];
      Simulator_DW.Memory_PreviousInput_a5[1] = Simulator_B.LogicalOperator_b[1];
      Simulator_DW.Memory_PreviousInput_a5[2] = Simulator_B.LogicalOperator_b[2];

      /* Update for Memory: '<S63>/Memory' */
      Simulator_DW.Memory_PreviousInput_n[0] = Simulator_B.LogicalOperator_o[0];
      Simulator_DW.Memory_PreviousInput_n[1] = Simulator_B.LogicalOperator_o[1];
      Simulator_DW.Memory_PreviousInput_n[2] = Simulator_B.LogicalOperator_o[2];
    }

    /* Update for Integrator: '<S20>/Integrator' */
    Simulator_DW.Integrator_IWORK_p = 0;

    /* Update for Integrator: '<S60>/Integrator' */
    Simulator_DW.Integrator_IWORK_c = 0;
    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[2] == 0) {
      /* Update for RandomNumber: '<S271>/White Noise' */
      Simulator_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&Simulator_DW.RandSeed[0]);
      Simulator_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&Simulator_DW.RandSeed[1]);
      Simulator_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&Simulator_DW.RandSeed[2]);
      Simulator_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&Simulator_DW.RandSeed[3]);
    }

    if (rtmIsMajorTimeStep((&Simulator_M)) &&
        (&Simulator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Update for DiscreteIntegrator: '<S81>/Integrator' */
      Simulator_DW.Integrator_PrevResetState = 0;
      for (i = 0; i < 5; i++) {
        Simulator_DW.Integrator_DSTATE[i] += 0.001 * rtb_Integrator[i];

        /* Update for DiscreteIntegrator: '<S82>/Integrator' */
        Simulator_DW.Integrator_DSTATE_m[i] += 0.001 * rtb_Sum_b[i];
      }

      /* End of Update for DiscreteIntegrator: '<S81>/Integrator' */

      /* Update for DiscreteIntegrator: '<S82>/Integrator' */
      Simulator_DW.Integrator_PrevResetState_c = 0;

      /* Update for DiscreteIntegrator: '<S77>/Integrator' */
      Simulator_DW.Integrator_PrevResetState_g = 0;
      Simulator_DW.Integrator_DSTATE_b[0] += 0.001 * rtb_Integrator_f[0];

      /* Update for DiscreteIntegrator: '<S78>/Integrator' */
      Simulator_DW.Integrator_DSTATE_c[0] += 0.001 * rtb_Sum_f[0];

      /* Update for DiscreteIntegrator: '<S77>/Integrator' */
      Simulator_DW.Integrator_DSTATE_b[1] += 0.001 * rtb_Integrator_f[1];

      /* Update for DiscreteIntegrator: '<S78>/Integrator' */
      Simulator_DW.Integrator_DSTATE_c[1] += 0.001 * rtb_Sum_f[1];
      Simulator_DW.Integrator_PrevResetState_l = 0;

      /* Update for Memory: '<S57>/Memory' */
      Simulator_DW.Memory_PreviousInput_g = Simulator_B.Logic[0];
    }

    /* ContTimeOutputInconsistentWithStateAtMajorOutputFlag is set, need to run a minor output */
    if (rtmIsMajorTimeStep((&Simulator_M))) {
      if (rtsiGetContTimeOutputInconsistentWithStateAtMajorStep(&(&Simulator_M
           )->solverInfo)) {
        rtsiSetSimTimeStep(&(&Simulator_M)->solverInfo,MINOR_TIME_STEP);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep(&(&Simulator_M)
          ->solverInfo, false);
        SimulatorModelClass::step();
        rtsiSetSimTimeStep(&(&Simulator_M)->solverInfo, MAJOR_TIME_STEP);
      }
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&Simulator_M))) {
    rt_ertODEUpdateContinuousStates(&(&Simulator_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&Simulator_M)->Timing.clockTick0)) {
      ++(&Simulator_M)->Timing.clockTickH0;
    }

    (&Simulator_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&Simulator_M)
      ->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&Simulator_M)->Timing.clockTick1++;
      if (!(&Simulator_M)->Timing.clockTick1) {
        (&Simulator_M)->Timing.clockTickH1++;
      }
    }

    rate_scheduler((&Simulator_M));
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void SimulatorModelClass::Simulator_derivatives()
{
  boolean_T lsat;
  boolean_T usat;
  XDot_Simulator_T *_rtXdot;
  _rtXdot = ((XDot_Simulator_T *) (&Simulator_M)->derivs);

  /* Derivatives for Integrator: '<S205>/Integrator' incorporates:
   *  Constant: '<S205>/Constant'
   */
  _rtXdot->Integrator_CSTATE = 1.0;

  /* Derivatives for Integrator: '<S8>/Integrator' incorporates:
   *  Constant: '<S8>/Constant3'
   */
  if (!Simulator_B.NOT) {
    _rtXdot->Integrator_CSTATE_h = 1.0;
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_h = 0.0;
  }

  /* End of Derivatives for Integrator: '<S8>/Integrator' */

  /* Derivatives for Integrator: '<S8>/Integrator2' */
  _rtXdot->Integrator2_CSTATE = Simulator_B.Vx;

  /* Derivatives for Integrator: '<S8>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = Simulator_B.Vy;

  /* Derivatives for Integrator: '<S8>/Integrator3' */
  _rtXdot->Integrator3_CSTATE = Simulator_B.Vz;

  /* Derivatives for Integrator: '<S45>/Integrator' */
  if (!Simulator_B.LogicalOperator4[0]) {
    _rtXdot->Integrator_CSTATE_o[0] = Simulator_B.Switch_i[0];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_o[0] = 0.0;
  }

  /* Derivatives for Integrator: '<S37>/Integrator' */
  if (!Simulator_B.LogicalOperator4_i[0]) {
    _rtXdot->Integrator_CSTATE_f[0] = Simulator_B.Switch_d[0];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_f[0] = 0.0;
  }

  /* Derivatives for Integrator: '<S20>/Integrator' */
  if (!Simulator_B.LogicalOperator4_d[0]) {
    _rtXdot->Integrator_CSTATE_b[0] = Simulator_B.Switch[0];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_b[0] = 0.0;
  }

  /* Derivatives for Integrator: '<S60>/Integrator' */
  if (!Simulator_B.LogicalOperator4_a[0]) {
    _rtXdot->Integrator_CSTATE_a[0] = Simulator_B.Switch_j[0];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_a[0] = 0.0;
  }

  /* Derivatives for Integrator: '<S45>/Integrator' */
  if (!Simulator_B.LogicalOperator4[1]) {
    _rtXdot->Integrator_CSTATE_o[1] = Simulator_B.Switch_i[1];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_o[1] = 0.0;
  }

  /* Derivatives for Integrator: '<S37>/Integrator' */
  if (!Simulator_B.LogicalOperator4_i[1]) {
    _rtXdot->Integrator_CSTATE_f[1] = Simulator_B.Switch_d[1];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_f[1] = 0.0;
  }

  /* Derivatives for Integrator: '<S20>/Integrator' */
  if (!Simulator_B.LogicalOperator4_d[1]) {
    _rtXdot->Integrator_CSTATE_b[1] = Simulator_B.Switch[1];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_b[1] = 0.0;
  }

  /* Derivatives for Integrator: '<S60>/Integrator' */
  if (!Simulator_B.LogicalOperator4_a[1]) {
    _rtXdot->Integrator_CSTATE_a[1] = Simulator_B.Switch_j[1];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_a[1] = 0.0;
  }

  /* Derivatives for Integrator: '<S45>/Integrator' */
  if (!Simulator_B.LogicalOperator4[2]) {
    _rtXdot->Integrator_CSTATE_o[2] = Simulator_B.Switch_i[2];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_o[2] = 0.0;
  }

  /* Derivatives for Integrator: '<S37>/Integrator' */
  if (!Simulator_B.LogicalOperator4_i[2]) {
    _rtXdot->Integrator_CSTATE_f[2] = Simulator_B.Switch_d[2];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_f[2] = 0.0;
  }

  /* Derivatives for Integrator: '<S20>/Integrator' */
  if (!Simulator_B.LogicalOperator4_d[2]) {
    _rtXdot->Integrator_CSTATE_b[2] = Simulator_B.Switch[2];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_b[2] = 0.0;
  }

  /* Derivatives for Integrator: '<S60>/Integrator' */
  if (!Simulator_B.LogicalOperator4_a[2]) {
    _rtXdot->Integrator_CSTATE_a[2] = Simulator_B.Switch_j[2];
  } else {
    /* level reset is active */
    _rtXdot->Integrator_CSTATE_a[2] = 0.0;
  }

  /* Derivatives for Enabled SubSystem: '<S262>/Hugw(s)' */
  if (Simulator_DW.Hugws_MODE) {
    /* Derivatives for Integrator: '<S275>/ug_p' */
    _rtXdot->ug_p_CSTATE[0] = Simulator_B.w_e[0];
    _rtXdot->ug_p_CSTATE[1] = Simulator_B.w_e[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Simulator_T *) (&Simulator_M)->derivs)->ug_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S262>/Hugw(s)' */

  /* Derivatives for Enabled SubSystem: '<S262>/Hvgw(s)' */
  if (Simulator_DW.Hvgws_MODE) {
    /* Derivatives for Integrator: '<S276>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[0] = Simulator_B.w_h[0];

    /* Derivatives for Integrator: '<S276>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[0] = Simulator_B.w_j[0];

    /* Derivatives for Integrator: '<S276>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[1] = Simulator_B.w_h[1];

    /* Derivatives for Integrator: '<S276>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[1] = Simulator_B.w_j[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Simulator_T *) (&Simulator_M)->derivs)->vg_p1_CSTATE[0]);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S262>/Hvgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S262>/Hwgw(s)' */
  if (Simulator_DW.Hwgws_MODE) {
    /* Derivatives for Integrator: '<S277>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[0] = Simulator_B.w[0];

    /* Derivatives for Integrator: '<S277>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[0] = Simulator_B.w_k[0];

    /* Derivatives for Integrator: '<S277>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[1] = Simulator_B.w[1];

    /* Derivatives for Integrator: '<S277>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[1] = Simulator_B.w_k[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Simulator_T *) (&Simulator_M)->derivs)->wg_p1_CSTATE[0]);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S262>/Hwgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S253>/Distance into gust (x)' */
  if (Simulator_DW.Distanceintogustx_MODE) {
    /* Derivatives for Integrator: '<S256>/Distance into Gust (x) (Limited to gust length d)' */
    lsat = (Simulator_X.DistanceintoGustxLimitedtogus_n <= 0.0);
    usat = (Simulator_X.DistanceintoGustxLimitedtogus_n >= 120.0);
    if (((!lsat) && (!usat)) || (lsat && (Simulator_B.Sqrt > 0.0)) || (usat &&
         (Simulator_B.Sqrt < 0.0))) {
      _rtXdot->DistanceintoGustxLimitedtogus_n = Simulator_B.Sqrt;
    } else {
      /* in saturation */
      _rtXdot->DistanceintoGustxLimitedtogus_n = 0.0;
    }

    /* End of Derivatives for Integrator: '<S256>/Distance into Gust (x) (Limited to gust length d)' */
  } else {
    ((XDot_Simulator_T *) (&Simulator_M)->derivs)
      ->DistanceintoGustxLimitedtogus_n = 0.0;
  }

  /* End of Derivatives for SubSystem: '<S253>/Distance into gust (x)' */

  /* Derivatives for Enabled SubSystem: '<S253>/Distance into gust (y)' */
  Simulat_Distanceintogusty_Deriv(Simulator_B.Sqrt,
    &Simulator_DW.Distanceintogusty, &Simulator_X.Distanceintogusty,
    &_rtXdot->Distanceintogusty, 120.0);

  /* End of Derivatives for SubSystem: '<S253>/Distance into gust (y)' */

  /* Derivatives for Enabled SubSystem: '<S253>/Distance into gust (z)' */
  Simulat_Distanceintogusty_Deriv(Simulator_B.Sqrt,
    &Simulator_DW.Distanceintogustz, &Simulator_X.Distanceintogustz,
    &_rtXdot->Distanceintogustz, 80.0);

  /* End of Derivatives for SubSystem: '<S253>/Distance into gust (z)' */

  /* Derivatives for Enabled SubSystem: '<S261>/Hpgw' */
  if (Simulator_DW.Hpgw_MODE) {
    /* Derivatives for Integrator: '<S272>/pgw_p' */
    _rtXdot->pgw_p_CSTATE[0] = Simulator_B.w_hh[0];
    _rtXdot->pgw_p_CSTATE[1] = Simulator_B.w_hh[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Simulator_T *) (&Simulator_M)->derivs)->pgw_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S261>/Hpgw' */

  /* Derivatives for Enabled SubSystem: '<S261>/Hqgw' */
  if (Simulator_DW.Hqgw_MODE) {
    /* Derivatives for Integrator: '<S273>/qgw_p' */
    _rtXdot->qgw_p_CSTATE[0] = Simulator_B.w_b[0];
    _rtXdot->qgw_p_CSTATE[1] = Simulator_B.w_b[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Simulator_T *) (&Simulator_M)->derivs)->qgw_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S261>/Hqgw' */

  /* Derivatives for Enabled SubSystem: '<S261>/Hrgw' */
  if (Simulator_DW.Hrgw_MODE) {
    /* Derivatives for Integrator: '<S274>/rgw_p' */
    _rtXdot->rgw_p_CSTATE[0] = Simulator_B.w_jk[0];
    _rtXdot->rgw_p_CSTATE[1] = Simulator_B.w_jk[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Simulator_T *) (&Simulator_M)->derivs)->rgw_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S261>/Hrgw' */
}

/* Model initialize function */
void SimulatorModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&Simulator_M)->solverInfo, &(&Simulator_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&Simulator_M)->solverInfo, &rtmGetTPtr((&Simulator_M)));
    rtsiSetStepSizePtr(&(&Simulator_M)->solverInfo, &(&Simulator_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&Simulator_M)->solverInfo, &(&Simulator_M)->derivs);
    rtsiSetContStatesPtr(&(&Simulator_M)->solverInfo, (real_T **) &(&Simulator_M)
                         ->contStates);
    rtsiSetNumContStatesPtr(&(&Simulator_M)->solverInfo, &(&Simulator_M)
      ->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&Simulator_M)->solverInfo, &(&Simulator_M
      )->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&Simulator_M)->solverInfo,
      &(&Simulator_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&Simulator_M)->solverInfo,
      &(&Simulator_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&Simulator_M)->solverInfo, (&rtmGetErrorStatus
      ((&Simulator_M))));
    rtsiSetRTModelPtr(&(&Simulator_M)->solverInfo, (&Simulator_M));
  }

  rtsiSetSimTimeStep(&(&Simulator_M)->solverInfo, MAJOR_TIME_STEP);
  (&Simulator_M)->intgData.f[0] = (&Simulator_M)->odeF[0];
  (&Simulator_M)->contStates = ((X_Simulator_T *) &Simulator_X);
  (&Simulator_M)->periodicContStateIndices = ((int_T*) Simulator_PeriodicIndX);
  (&Simulator_M)->periodicContStateRanges = ((real_T*) Simulator_PeriodicRngX);
  rtsiSetSolverData(&(&Simulator_M)->solverInfo, static_cast<void *>
                    (&(&Simulator_M)->intgData));
  rtsiSetSolverName(&(&Simulator_M)->solverInfo,"ode1");
  rtmSetTPtr((&Simulator_M), &(&Simulator_M)->Timing.tArray[0]);
  (&Simulator_M)->Timing.stepSize0 = 0.001;
  rtmSetFirstInitCond((&Simulator_M), 1);

  /* block I/O */
  (void) std::memset((static_cast<void *>(&Simulator_B)), 0,
                     sizeof(B_Simulator_T));

  {
    int32_T i;
    for (i = 0; i < 9; i++) {
      Simulator_B.VectorConcatenate[i] = 0.0;
    }

    for (i = 0; i < 5; i++) {
      Simulator_B.Add1[i] = 0.0;
    }

    Simulator_B.Switch3[0] = 0.0;
    Simulator_B.Switch3[1] = 0.0;
    Simulator_B.Switch3[2] = 0.0;
    Simulator_B.Switch2[0] = 0.0;
    Simulator_B.Switch2[1] = 0.0;
    Simulator_B.Switch2[2] = 0.0;
    Simulator_B.Switch8[0] = 0.0;
    Simulator_B.Switch8[1] = 0.0;
    Simulator_B.Switch8[2] = 0.0;
    Simulator_B.Switch2_a[0] = 0.0;
    Simulator_B.Switch2_a[1] = 0.0;
    Simulator_B.Switch2_a[2] = 0.0;
    Simulator_B.Switch1[0] = 0.0;
    Simulator_B.Switch1[1] = 0.0;
    Simulator_B.Switch1[2] = 0.0;
    Simulator_B.Vx = 0.0;
    Simulator_B.Vy = 0.0;
    Simulator_B.Vz = 0.0;
    Simulator_B.Switch2_n[0] = 0.0;
    Simulator_B.Switch2_n[1] = 0.0;
    Simulator_B.Switch2_n[2] = 0.0;
    Simulator_B.Switch4[0] = 0.0;
    Simulator_B.Switch4[1] = 0.0;
    Simulator_B.Switch4[2] = 0.0;
    Simulator_B.Switch2_f[0] = 0.0;
    Simulator_B.Switch2_f[1] = 0.0;
    Simulator_B.Switch2_f[2] = 0.0;
    Simulator_B.Add[0] = 0.0;
    Simulator_B.Add[1] = 0.0;
    Simulator_B.Add[2] = 0.0;
    Simulator_B.SFunction_o1 = 0.0;
    Simulator_B.SFunction_o2 = 0.0;
    Simulator_B.SFunction_o3 = 0.0;
    Simulator_B.SFunction_o4 = 0.0;
    Simulator_B.Sqrt = 0.0;
    Simulator_B.Product[0] = 0.0;
    Simulator_B.Product[1] = 0.0;
    Simulator_B.Product[2] = 0.0;
    Simulator_B.Product[3] = 0.0;
    Simulator_B.Constant3 = 0.0;
    Simulator_B.Constant2 = 0.0;
    Simulator_B.Constant3_i = 0.0;
    Simulator_B.Constant2_m = 0.0;
    Simulator_B.Add_j[0] = 0.0;
    Simulator_B.Add_j[1] = 0.0;
    Simulator_B.Merge[0] = 0.0;
    Simulator_B.Merge[1] = 0.0;
    Simulator_B.Merge[2] = 0.0;
    Simulator_B.Merge[3] = 0.0;
    Simulator_B.Clock = 0.0;
    Simulator_B.Switch[0] = 0.0;
    Simulator_B.Switch[1] = 0.0;
    Simulator_B.Switch[2] = 0.0;
    Simulator_B.Clock_o = 0.0;
    Simulator_B.Switch_d[0] = 0.0;
    Simulator_B.Switch_d[1] = 0.0;
    Simulator_B.Switch_d[2] = 0.0;
    Simulator_B.Clock_i = 0.0;
    Simulator_B.Switch_i[0] = 0.0;
    Simulator_B.Switch_i[1] = 0.0;
    Simulator_B.Switch_i[2] = 0.0;
    Simulator_B.Clock_c = 0.0;
    Simulator_B.Switch_j[0] = 0.0;
    Simulator_B.Switch_j[1] = 0.0;
    Simulator_B.Switch_j[2] = 0.0;
    Simulator_B.w[0] = 0.0;
    Simulator_B.w[1] = 0.0;
    Simulator_B.LwgV1[0] = 0.0;
    Simulator_B.LwgV1[1] = 0.0;
    Simulator_B.w_k[0] = 0.0;
    Simulator_B.w_k[1] = 0.0;
    Simulator_B.w_h[0] = 0.0;
    Simulator_B.w_h[1] = 0.0;
    Simulator_B.w_j[0] = 0.0;
    Simulator_B.w_j[1] = 0.0;
    Simulator_B.w1[0] = 0.0;
    Simulator_B.w1[1] = 0.0;
    Simulator_B.w_e[0] = 0.0;
    Simulator_B.w_e[1] = 0.0;
    Simulator_B.w1_p[0] = 0.0;
    Simulator_B.w1_p[1] = 0.0;
    Simulator_B.w_jk[0] = 0.0;
    Simulator_B.w_jk[1] = 0.0;
    Simulator_B.UnaryMinus[0] = 0.0;
    Simulator_B.UnaryMinus[1] = 0.0;
    Simulator_B.w_b[0] = 0.0;
    Simulator_B.w_b[1] = 0.0;
    Simulator_B.sigma_w[0] = 0.0;
    Simulator_B.sigma_w[1] = 0.0;
    Simulator_B.w_hh[0] = 0.0;
    Simulator_B.w_hh[1] = 0.0;
    Simulator_B.DistanceintoGustxLimitedtogustl = 0.0;
    Simulator_B.Distanceintogustz.DistanceintoGustxLimitedtogustl = 0.0;
    Simulator_B.Distanceintogusty.DistanceintoGustxLimitedtogustl = 0.0;
  }

  /* states (continuous) */
  {
    (void) std::memset(static_cast<void *>(&Simulator_X), 0,
                       sizeof(X_Simulator_T));
  }

  /* Periodic continuous states */
  {
    (void) std::memset(static_cast<void*>(Simulator_PeriodicIndX), 0,
                       3*sizeof(int_T));
    (void) std::memset(static_cast<void*>(Simulator_PeriodicRngX), 0,
                       6*sizeof(real_T));
  }

  /* states (dwork) */
  (void) std::memset(static_cast<void *>(&Simulator_DW), 0,
                     sizeof(DW_Simulator_T));

  {
    int32_T i;
    for (i = 0; i < 5; i++) {
      Simulator_DW.Integrator_DSTATE[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 5; i++) {
      Simulator_DW.Integrator_DSTATE_m[i] = 0.0;
    }
  }

  Simulator_DW.Integrator_DSTATE_b[0] = 0.0;
  Simulator_DW.Integrator_DSTATE_b[1] = 0.0;
  Simulator_DW.Integrator_DSTATE_c[0] = 0.0;
  Simulator_DW.Integrator_DSTATE_c[1] = 0.0;

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Simulator_DW.SFunction_temp_table[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Simulator_DW.SFunction_pres_table[i] = 0.0;
    }
  }

  Simulator_DW.NextOutput[0] = 0.0;
  Simulator_DW.NextOutput[1] = 0.0;
  Simulator_DW.NextOutput[2] = 0.0;
  Simulator_DW.NextOutput[3] = 0.0;

  {
    int32_T i;
    for (i = 0; i < 9; i++) {
      Simulator_DW.Product2_DWORK4[i] = 0.0;
    }
  }

  /* Start for S-Function (saeroatmos): '<S247>/S-Function' */
  {
    real_T *temp_table = (real_T *) &Simulator_DW.SFunction_temp_table[0];
    real_T *pres_table = (real_T *) &Simulator_DW.SFunction_pres_table[0];

    /* COESA */
    /*
     * Initialize COESA pressure and temperature tables.
     */
    InitCalcAtmosCOESA( temp_table, pres_table );
  }

  /* Start for If: '<S267>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  Simulator_DW.ifHeightMaxlowaltitudeelseifHei = -1;

  /* Start for Constant: '<S74>/Constant3' */
  Simulator_B.Constant3 = 0.0;

  /* Start for Constant: '<S74>/Constant2' */
  Simulator_B.Constant2 = 0.0;

  /* Start for Constant: '<S73>/Constant3' */
  Simulator_B.Constant3_i = 0.0;

  /* Start for Constant: '<S73>/Constant2' */
  Simulator_B.Constant2_m = 0.0;

  /* Start for If: '<S266>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  Simulator_DW.ifHeightMaxlowaltitudeelseifH_e = -1;

  /* Start for If: '<S206>/If' */
  Simulator_DW.If_ActiveSubsystem = -1;

  /* Start for IfAction SubSystem: '<S206>/Negative Trace' */
  /* Start for If: '<S207>/Find Maximum Diagonal Value' */
  Simulator_DW.FindMaximumDiagonalValue_Active = -1;

  /* End of Start for SubSystem: '<S206>/Negative Trace' */

  /* Start for If: '<S209>/If1' */
  Simulator_DW.If1_ActiveSubsystem = -1;

  /* Start for DataStoreMemory: '<S205>/Data Store Memory' */
  Simulator_DW.CmdBus_e = Simulator_rtZCmdBus;

  /* Start for DataStoreMemory: '<S205>/Data Store Memory1' */
  Simulator_DW.ACBus_o = Simulator_rtZACBus;
  Simulator_PrevZCX.Integrator_Reset_ZCE = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_m[0] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_g[0] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_my[0] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_c[0] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem_h.TriggeredSubsystem_Trig_ZCE[0] =
    POS_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem_g.TriggeredSubsystem_Trig_ZCE[0] =
    POS_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem_m.TriggeredSubsystem_Trig_ZCE[0] =
    POS_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem.TriggeredSubsystem_Trig_ZCE[0] =
    POS_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_m[1] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_g[1] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_my[1] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_c[1] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem_h.TriggeredSubsystem_Trig_ZCE[1] =
    POS_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem_g.TriggeredSubsystem_Trig_ZCE[1] =
    POS_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem_m.TriggeredSubsystem_Trig_ZCE[1] =
    POS_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem.TriggeredSubsystem_Trig_ZCE[1] =
    POS_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_m[2] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_g[2] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_my[2] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.Integrator_Reset_ZCE_c[2] = UNINITIALIZED_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem_h.TriggeredSubsystem_Trig_ZCE[2] =
    POS_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem_g.TriggeredSubsystem_Trig_ZCE[2] =
    POS_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem_m.TriggeredSubsystem_Trig_ZCE[2] =
    POS_ZCSIG;
  Simulator_PrevZCX.TriggeredSubsystem.TriggeredSubsystem_Trig_ZCE[2] =
    POS_ZCSIG;

  {
    int32_T i;

    /* InitializeConditions for Integrator: '<S205>/Integrator' */
    Simulator_X.Integrator_CSTATE = 0.0;

    /* InitializeConditions for Memory: '<S55>/Memory' */
    Simulator_DW.Memory_PreviousInput = false;

    /* InitializeConditions for Memory: '<S8>/Memory' */
    Simulator_DW.Memory_PreviousInput_f = false;

    /* InitializeConditions for Integrator: '<S8>/Integrator' */
    Simulator_X.Integrator_CSTATE_h = 0.0;

    /* InitializeConditions for Memory: '<S48>/Memory' */
    Simulator_DW.Memory_PreviousInput_l[0] = false;
    Simulator_DW.Memory_PreviousInput_l[1] = false;
    Simulator_DW.Memory_PreviousInput_l[2] = false;

    /* InitializeConditions for Integrator: '<S45>/Integrator' incorporates:
     *  Integrator: '<S37>/Integrator'
     */
    if (rtmIsFirstInitCond((&Simulator_M))) {
      Simulator_X.Integrator_CSTATE_o[0] = 0.0;
      Simulator_X.Integrator_CSTATE_o[1] = 0.0;
      Simulator_X.Integrator_CSTATE_o[2] = 0.0;
      Simulator_X.Integrator_CSTATE_f[0] = 0.0;
      Simulator_X.Integrator_CSTATE_f[1] = 0.0;
      Simulator_X.Integrator_CSTATE_f[2] = 0.0;
    }

    Simulator_DW.Integrator_IWORK = 1;

    /* End of InitializeConditions for Integrator: '<S45>/Integrator' */

    /* InitializeConditions for Memory: '<S40>/Memory' */
    Simulator_DW.Memory_PreviousInput_a[0] = false;
    Simulator_DW.Memory_PreviousInput_a[1] = false;
    Simulator_DW.Memory_PreviousInput_a[2] = false;

    /* InitializeConditions for Integrator: '<S37>/Integrator' */
    Simulator_DW.Integrator_IWORK_o = 1;

    /* InitializeConditions for Memory: '<S23>/Memory' */
    Simulator_DW.Memory_PreviousInput_a5[0] = false;
    Simulator_DW.Memory_PreviousInput_a5[1] = false;
    Simulator_DW.Memory_PreviousInput_a5[2] = false;

    /* InitializeConditions for Integrator: '<S20>/Integrator' incorporates:
     *  Integrator: '<S60>/Integrator'
     */
    if (rtmIsFirstInitCond((&Simulator_M))) {
      Simulator_X.Integrator_CSTATE_b[0] = 0.0;
      Simulator_X.Integrator_CSTATE_b[1] = 0.0;
      Simulator_X.Integrator_CSTATE_b[2] = 0.0;
      Simulator_X.Integrator_CSTATE_a[0] = 0.0;
      Simulator_X.Integrator_CSTATE_a[1] = 0.0;
      Simulator_X.Integrator_CSTATE_a[2] = 0.0;
    }

    Simulator_DW.Integrator_IWORK_p = 1;

    /* End of InitializeConditions for Integrator: '<S20>/Integrator' */

    /* InitializeConditions for Memory: '<S63>/Memory' */
    Simulator_DW.Memory_PreviousInput_n[0] = false;
    Simulator_DW.Memory_PreviousInput_n[1] = false;
    Simulator_DW.Memory_PreviousInput_n[2] = false;

    /* InitializeConditions for Integrator: '<S8>/Integrator2' */
    Simulator_X.Integrator2_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S8>/Integrator1' */
    Simulator_X.Integrator1_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S8>/Integrator3' */
    Simulator_X.Integrator3_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S60>/Integrator' */
    Simulator_DW.Integrator_IWORK_c = 1;

    /* InitializeConditions for RandomNumber: '<S271>/White Noise' */
    Simulator_DW.RandSeed[0] = 65536U;
    Simulator_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
      (&Simulator_DW.RandSeed[0]);
    Simulator_DW.RandSeed[1] = 131072U;
    Simulator_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
      (&Simulator_DW.RandSeed[1]);
    Simulator_DW.RandSeed[2] = 196608U;
    Simulator_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
      (&Simulator_DW.RandSeed[2]);
    Simulator_DW.RandSeed[3] = 262144U;
    Simulator_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
      (&Simulator_DW.RandSeed[3]);

    /* InitializeConditions for DiscreteIntegrator: '<S81>/Integrator' */
    Simulator_DW.Integrator_PrevResetState = 0;
    for (i = 0; i < 5; i++) {
      Simulator_DW.Integrator_DSTATE[i] = Simulator_B.Constant3;

      /* InitializeConditions for DiscreteIntegrator: '<S82>/Integrator' */
      Simulator_DW.Integrator_DSTATE_m[i] = Simulator_B.Constant2;
    }

    /* End of InitializeConditions for DiscreteIntegrator: '<S81>/Integrator' */

    /* InitializeConditions for DiscreteIntegrator: '<S82>/Integrator' */
    Simulator_DW.Integrator_PrevResetState_c = 0;

    /* InitializeConditions for DiscreteIntegrator: '<S77>/Integrator' */
    Simulator_DW.Integrator_PrevResetState_g = 0;
    Simulator_DW.Integrator_DSTATE_b[0] = Simulator_B.Constant3_i;

    /* InitializeConditions for DiscreteIntegrator: '<S78>/Integrator' */
    Simulator_DW.Integrator_DSTATE_c[0] = Simulator_B.Constant2_m;

    /* InitializeConditions for DiscreteIntegrator: '<S77>/Integrator' */
    Simulator_DW.Integrator_DSTATE_b[1] = Simulator_B.Constant3_i;

    /* InitializeConditions for DiscreteIntegrator: '<S78>/Integrator' */
    Simulator_DW.Integrator_DSTATE_c[1] = Simulator_B.Constant2_m;
    Simulator_DW.Integrator_PrevResetState_l = 0;

    /* InitializeConditions for Memory: '<S57>/Memory' */
    Simulator_DW.Memory_PreviousInput_g = false;

    /* SystemInitialize for Enabled SubSystem: '<S48>/POSITIVE Edge' */
    Simulator_POSITIVEEdge_Init(&Simulator_B.POSITIVEEdge_o0);

    /* End of SystemInitialize for SubSystem: '<S48>/POSITIVE Edge' */

    /* SystemInitialize for Enabled SubSystem: '<S48>/NEGATIVE Edge' */
    Simulator_NEGATIVEEdge_Init(&Simulator_B.NEGATIVEEdge_j);

    /* End of SystemInitialize for SubSystem: '<S48>/NEGATIVE Edge' */

    /* SystemInitialize for Enabled SubSystem: '<S40>/POSITIVE Edge' */
    Simulator_POSITIVEEdge_Init(&Simulator_B.POSITIVEEdge_o);

    /* End of SystemInitialize for SubSystem: '<S40>/POSITIVE Edge' */

    /* SystemInitialize for Enabled SubSystem: '<S40>/NEGATIVE Edge' */
    Simulator_NEGATIVEEdge_Init(&Simulator_B.NEGATIVEEdge_f);

    /* End of SystemInitialize for SubSystem: '<S40>/NEGATIVE Edge' */

    /* SystemInitialize for Enabled SubSystem: '<S23>/POSITIVE Edge' */
    Simulator_POSITIVEEdge_Init(&Simulator_B.POSITIVEEdge);

    /* End of SystemInitialize for SubSystem: '<S23>/POSITIVE Edge' */

    /* SystemInitialize for Enabled SubSystem: '<S23>/NEGATIVE Edge' */
    Simulator_NEGATIVEEdge_Init(&Simulator_B.NEGATIVEEdge);

    /* End of SystemInitialize for SubSystem: '<S23>/NEGATIVE Edge' */

    /* SystemInitialize for Enabled SubSystem: '<S63>/POSITIVE Edge' */
    Simulator_POSITIVEEdge_Init(&Simulator_B.POSITIVEEdge_l);

    /* End of SystemInitialize for SubSystem: '<S63>/POSITIVE Edge' */

    /* SystemInitialize for Enabled SubSystem: '<S63>/NEGATIVE Edge' */
    Simulator_NEGATIVEEdge_Init(&Simulator_B.NEGATIVEEdge_fy);

    /* End of SystemInitialize for SubSystem: '<S63>/NEGATIVE Edge' */

    /* SystemInitialize for Enabled SubSystem: '<S262>/Hugw(s)' */
    /* InitializeConditions for Integrator: '<S275>/ug_p' */
    Simulator_X.ug_p_CSTATE[0] = 0.0;

    /* SystemInitialize for Outport: '<S275>/ugw' */
    Simulator_B.w1_p[0] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S262>/Hugw(s)' */

    /* SystemInitialize for Enabled SubSystem: '<S262>/Hvgw(s)' */
    /* InitializeConditions for Integrator: '<S276>/vg_p1' */
    Simulator_X.vg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S276>/vgw_p2' */
    Simulator_X.vgw_p2_CSTATE[0] = 0.0;

    /* SystemInitialize for Outport: '<S276>/vgw' */
    Simulator_B.w1[0] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S262>/Hvgw(s)' */

    /* SystemInitialize for Enabled SubSystem: '<S262>/Hwgw(s)' */
    /* InitializeConditions for Integrator: '<S277>/wg_p1' */
    Simulator_X.wg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S277>/wg_p2' */
    Simulator_X.wg_p2_CSTATE[0] = 0.0;

    /* SystemInitialize for Outport: '<S277>/wgw' */
    Simulator_B.LwgV1[0] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S262>/Hwgw(s)' */

    /* SystemInitialize for Enabled SubSystem: '<S262>/Hugw(s)' */
    /* InitializeConditions for Integrator: '<S275>/ug_p' */
    Simulator_X.ug_p_CSTATE[1] = 0.0;

    /* SystemInitialize for Outport: '<S275>/ugw' */
    Simulator_B.w1_p[1] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S262>/Hugw(s)' */

    /* SystemInitialize for Enabled SubSystem: '<S262>/Hvgw(s)' */
    /* InitializeConditions for Integrator: '<S276>/vg_p1' */
    Simulator_X.vg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S276>/vgw_p2' */
    Simulator_X.vgw_p2_CSTATE[1] = 0.0;

    /* SystemInitialize for Outport: '<S276>/vgw' */
    Simulator_B.w1[1] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S262>/Hvgw(s)' */

    /* SystemInitialize for Enabled SubSystem: '<S262>/Hwgw(s)' */
    /* InitializeConditions for Integrator: '<S277>/wg_p1' */
    Simulator_X.wg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S277>/wg_p2' */
    Simulator_X.wg_p2_CSTATE[1] = 0.0;

    /* SystemInitialize for Outport: '<S277>/wgw' */
    Simulator_B.LwgV1[1] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S262>/Hwgw(s)' */

    /* SystemInitialize for Enabled SubSystem: '<S253>/Distance into gust (x)' */
    /* InitializeConditions for Integrator: '<S256>/Distance into Gust (x) (Limited to gust length d)' */
    Simulator_X.DistanceintoGustxLimitedtogus_n = 0.0;

    /* SystemInitialize for Outport: '<S256>/x' */
    Simulator_B.DistanceintoGustxLimitedtogustl = 0.0;

    /* End of SystemInitialize for SubSystem: '<S253>/Distance into gust (x)' */

    /* SystemInitialize for Enabled SubSystem: '<S253>/Distance into gust (y)' */
    Simulato_Distanceintogusty_Init(&Simulator_B.Distanceintogusty,
      &Simulator_X.Distanceintogusty);

    /* End of SystemInitialize for SubSystem: '<S253>/Distance into gust (y)' */

    /* SystemInitialize for Enabled SubSystem: '<S253>/Distance into gust (z)' */
    Simulato_Distanceintogusty_Init(&Simulator_B.Distanceintogustz,
      &Simulator_X.Distanceintogustz);

    /* End of SystemInitialize for SubSystem: '<S253>/Distance into gust (z)' */

    /* SystemInitialize for Enabled SubSystem: '<S261>/Hpgw' */
    /* InitializeConditions for Integrator: '<S272>/pgw_p' */
    Simulator_X.pgw_p_CSTATE[0] = 0.0;

    /* SystemInitialize for Outport: '<S272>/pgw' */
    Simulator_B.sigma_w[0] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S261>/Hpgw' */

    /* SystemInitialize for Enabled SubSystem: '<S261>/Hqgw' */
    /* InitializeConditions for Integrator: '<S273>/qgw_p' */
    Simulator_X.qgw_p_CSTATE[0] = 0.0;

    /* SystemInitialize for Outport: '<S273>/qgw' */
    Simulator_B.w_b[0] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S261>/Hqgw' */

    /* SystemInitialize for Enabled SubSystem: '<S261>/Hrgw' */
    /* InitializeConditions for Integrator: '<S274>/rgw_p' */
    Simulator_X.rgw_p_CSTATE[0] = 0.0;

    /* SystemInitialize for Outport: '<S274>/rgw' */
    Simulator_B.UnaryMinus[0] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S261>/Hrgw' */

    /* SystemInitialize for Enabled SubSystem: '<S261>/Hpgw' */
    /* InitializeConditions for Integrator: '<S272>/pgw_p' */
    Simulator_X.pgw_p_CSTATE[1] = 0.0;

    /* SystemInitialize for Outport: '<S272>/pgw' */
    Simulator_B.sigma_w[1] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S261>/Hpgw' */

    /* SystemInitialize for Enabled SubSystem: '<S261>/Hqgw' */
    /* InitializeConditions for Integrator: '<S273>/qgw_p' */
    Simulator_X.qgw_p_CSTATE[1] = 0.0;

    /* SystemInitialize for Outport: '<S273>/qgw' */
    Simulator_B.w_b[1] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S261>/Hqgw' */

    /* SystemInitialize for Enabled SubSystem: '<S261>/Hrgw' */
    /* InitializeConditions for Integrator: '<S274>/rgw_p' */
    Simulator_X.rgw_p_CSTATE[1] = 0.0;

    /* SystemInitialize for Outport: '<S274>/rgw' */
    Simulator_B.UnaryMinus[1] = 0.0;

    /* End of SystemInitialize for SubSystem: '<S261>/Hrgw' */

    /* SystemInitialize for Merge: '<S206>/Merge' */
    Simulator_B.Merge[0] = 1.0;
    Simulator_B.Merge[1] = 0.0;
    Simulator_B.Merge[2] = 0.0;
    Simulator_B.Merge[3] = 0.0;

    /* InitializeConditions for root-level periodic continuous states */
    {
      int_T rootPeriodicContStateIndices[3] = { 5, 6, 7 };

      real_T rootPeriodicContStateRanges[6] = { -3.1415926535897931,
        3.1415926535897931, -3.1415926535897931, 3.1415926535897931,
        -3.1415926535897931, 3.1415926535897931 };

      (void) std::memcpy((void*)Simulator_PeriodicIndX,
                         rootPeriodicContStateIndices,
                         3*sizeof(int_T));
      (void) std::memcpy((void*)Simulator_PeriodicRngX,
                         rootPeriodicContStateRanges,
                         6*sizeof(real_T));
    }

    /* set "at time zero" to false */
    if (rtmIsFirstInitCond((&Simulator_M))) {
      rtmSetFirstInitCond((&Simulator_M), 0);
    }
  }
}

/* Model terminate function */
void SimulatorModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
SimulatorModelClass::SimulatorModelClass() : Simulator_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
SimulatorModelClass::~SimulatorModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_Simulator_T * SimulatorModelClass::getRTM()
{
  return (&Simulator_M);
}

#pragma GCC diagnostic pop
