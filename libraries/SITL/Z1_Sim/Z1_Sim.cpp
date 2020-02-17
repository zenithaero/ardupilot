
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include "defines.h"

/*
 * Z1_Sim.cpp
 *
 * Code generation for model "Z1_Sim".
 *
 * Model version              : 0.242
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C++ source code generated on : Mon Feb 17 09:51:43 2020
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Z1_Sim.h"
#include "Z1_Sim_private.h"

const CmdBus Z1_Sim_rtZCmdBus = {
  0.0,                                 /* thr */
  0.0,                                 /* ail */
  0.0,                                 /* elev */
  0.0,                                 /* rud */
  0.0,                                 /* flaps */
  0.0,                                 /* ailOut */
  0.0                                  /* thrDiff */
} ;                                    /* CmdBus ground */

const ACBus Z1_Sim_rtZACBus = {
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

static void rate_scheduler(RT_MODEL_Z1_Sim_T *const Z1_Sim_M);

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
static void rate_scheduler(RT_MODEL_Z1_Sim_T *const Z1_Sim_M)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (Z1_Sim_M->Timing.TaskCounters.TID[2])++;
  if ((Z1_Sim_M->Timing.TaskCounters.TID[2]) > 2) {/* Sample time: [0.1s, 0.0s] */
    Z1_Sim_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
void Z1_SimModelClass::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = static_cast<ODE1_IntgData *>(rtsiGetSolverData(si));
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 33;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  Z1_Sim_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * System initialize for enable system:
 *    '<S163>/Distance into gust (y)'
 *    '<S163>/Distance into gust (z)'
 */
void Z1_SimModelClass::Z1_Sim_Distanceintogusty_Init
  (B_Distanceintogusty_Z1_Sim_T *localB, X_Distanceintogusty_Z1_Sim_T *localX)
{
  /* InitializeConditions for Integrator: '<S167>/Distance into Gust (x) (Limited to gust length d) ' */
  localX->DistanceintoGustxLimitedtogustl = 0.0;

  /* SystemInitialize for Outport: '<S167>/x' */
  localB->DistanceintoGustxLimitedtogustl = 0.0;
}

/*
 * System reset for enable system:
 *    '<S163>/Distance into gust (y)'
 *    '<S163>/Distance into gust (z)'
 */
void Z1_SimModelClass::Z1_Sim_Distanceintogusty_Reset
  (X_Distanceintogusty_Z1_Sim_T *localX)
{
  /* InitializeConditions for Integrator: '<S167>/Distance into Gust (x) (Limited to gust length d) ' */
  localX->DistanceintoGustxLimitedtogustl = 0.0;
}

/*
 * Disable for enable system:
 *    '<S163>/Distance into gust (y)'
 *    '<S163>/Distance into gust (z)'
 */
void Z1_SimModelClass::Z1_Si_Distanceintogusty_Disable
  (DW_Distanceintogusty_Z1_Sim_T *localDW)
{
  localDW->Distanceintogusty_MODE = false;
}

/*
 * Outputs for enable system:
 *    '<S163>/Distance into gust (y)'
 *    '<S163>/Distance into gust (z)'
 */
void Z1_SimModelClass::Z1_Sim_Distanceintogusty(boolean_T rtu_Enable,
  B_Distanceintogusty_Z1_Sim_T *localB, DW_Distanceintogusty_Z1_Sim_T *localDW,
  X_Distanceintogusty_Z1_Sim_T *localX, real_T rtp_d_m)
{
  /* Outputs for Enabled SubSystem: '<S163>/Distance into gust (y)' incorporates:
   *  EnablePort: '<S167>/Enable'
   */
  if ((rtmIsMajorTimeStep((&Z1_Sim_M)) &&
       (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) && rtmIsMajorTimeStep
      ((&Z1_Sim_M))) {
    if (rtu_Enable) {
      if (!localDW->Distanceintogusty_MODE) {
        Z1_Sim_Distanceintogusty_Reset(localX);
        localDW->Distanceintogusty_MODE = true;
      }
    } else {
      if (localDW->Distanceintogusty_MODE) {
        Z1_Si_Distanceintogusty_Disable(localDW);
      }
    }
  }

  if (localDW->Distanceintogusty_MODE) {
    /* Integrator: '<S167>/Distance into Gust (x) (Limited to gust length d) ' */
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

    /* End of Integrator: '<S167>/Distance into Gust (x) (Limited to gust length d) ' */
  }

  /* End of Outputs for SubSystem: '<S163>/Distance into gust (y)' */
}

/*
 * Derivatives for enable system:
 *    '<S163>/Distance into gust (y)'
 *    '<S163>/Distance into gust (z)'
 */
void Z1_SimModelClass::Z1_Sim_Distanceintogusty_Deriv(real_T rtu_V,
  DW_Distanceintogusty_Z1_Sim_T *localDW, X_Distanceintogusty_Z1_Sim_T *localX,
  XDot_Distanceintogusty_Z1_Sim_T *localXdot, real_T rtp_d_m)
{
  boolean_T lsat;
  boolean_T usat;
  if (localDW->Distanceintogusty_MODE) {
    /* Derivatives for Integrator: '<S167>/Distance into Gust (x) (Limited to gust length d) ' */
    lsat = (localX->DistanceintoGustxLimitedtogustl <= 0.0);
    usat = (localX->DistanceintoGustxLimitedtogustl >= rtp_d_m);
    if (((!lsat) && (!usat)) || (lsat && (rtu_V > 0.0)) || (usat && (rtu_V < 0.0)))
    {
      localXdot->DistanceintoGustxLimitedtogustl = rtu_V;
    } else {
      /* in saturation */
      localXdot->DistanceintoGustxLimitedtogustl = 0.0;
    }

    /* End of Derivatives for Integrator: '<S167>/Distance into Gust (x) (Limited to gust length d) ' */
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
void Z1_SimModelClass::step()
{
  ZCEventType zcEvent;
  boolean_T rEQ0;
  real_T rtb_f_n;
  real_T rtb_Saturation;
  real_T rtb_InterpolationUsingPrelook_m;
  real_T rtb_referencearea;
  real_T rtb_Product_c;
  real_T rtb_Sum2_m[3];
  real_T rtb_sigma_ugsigma_vg;
  uint32_T rtb_k_b;
  real_T rtb_VectorConcatenate_m[9];
  real_T rtb_Sum_gb[3];
  uint32_T rtb_k;
  uint32_T rtb_k_e;
  real_T rtb_Product_na[6];
  real_T rtb_HighGainQuaternionNormaliza;
  real_T rtb_q0dot;
  real_T rtb_q1dot;
  real_T rtb_q2dot;
  real_T rtb_q3dot;
  real_T rtb_fcn2;
  real_T rtb_Product1_pxv[3];
  real_T rtb_ail;
  real_T rtb_elev;
  real_T rtb_rud;
  real_T rtb_flaps;
  real_T rtb_ailOut;
  int8_T rtAction;
  real_T rtb_CLtoCz;
  real_T rtb_fcn3;
  real_T rtb_UnitConversion_i;
  real_T frac[2];
  uint32_T bpIndex[2];
  real_T rtb_MediumHighAltitudeIntensity;
  real_T frac_0[3];
  uint32_T bpIndex_0[3];
  real_T rtb_InterpolationUsingPrelookup;
  real_T frac_1[3];
  uint32_T bpIndex_1[3];
  real_T rtb_InterpolationUsingPreloo_nk;
  real_T frac_2[3];
  uint32_T bpIndex_2[3];
  real_T frac_3[3];
  uint32_T bpIndex_3[3];
  real_T rtb_InterpolationUsingPrelook_a;
  real_T frac_4[3];
  uint32_T bpIndex_4[3];
  real_T rtb_InterpolationUsingPrelook_p;
  real_T frac_5[3];
  uint32_T bpIndex_5[3];
  real_T rtb_InterpolationUsingPrelook_h;
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
  real_T rtb_DCM_bs[9];
  real_T rtb_Sum_a_0[6];
  int32_T i;
  real_T rtb_Product_d_idx_2;
  real_T rtb_Product_d_idx_3;
  real_T rtb_Product_d_idx_4;
  real_T rtb_Product_d_idx_5;
  real_T rtb_Product_ee_idx_0;
  real_T rtb_Product_ee_idx_1;
  real_T rtb_Product_ee_idx_2;
  real_T rtb_Product_ee_idx_3;
  real_T rtb_Product_ee_idx_4;
  real_T rtb_Divide_idx_0;
  real_T rtb_Divide_idx_1;
  real_T rtb_Divide_idx_2;
  real_T rtb_Divide_idx_3;
  real_T rtb_Divide_idx_4;
  real_T rtb_f_f_tmp;
  real_T rtb_Saturation_tmp;
  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    /* set solver stop time */
    if (!((&Z1_Sim_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&Z1_Sim_M)->solverInfo, (((&Z1_Sim_M)
        ->Timing.clockTickH0 + 1) * (&Z1_Sim_M)->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&Z1_Sim_M)->solverInfo, (((&Z1_Sim_M)
        ->Timing.clockTick0 + 1) * (&Z1_Sim_M)->Timing.stepSize0 + (&Z1_Sim_M)
        ->Timing.clockTickH0 * (&Z1_Sim_M)->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&Z1_Sim_M))) {
    (&Z1_Sim_M)->Timing.t[0] = rtsiGetT(&(&Z1_Sim_M)->solverInfo);
  }

  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
    /* DataStoreRead: '<S114>/Data Store Read' */
    Z1_Sim_B.DataStoreRead = Z1_Sim_DW.CmdBus_e;

    /* Gain: '<S114>/Gain' */
    rtb_ail = 45.0 * Z1_Sim_B.DataStoreRead.ail;

    /* Gain: '<S114>/Gain1' */
    rtb_elev = 45.0 * Z1_Sim_B.DataStoreRead.elev;

    /* Gain: '<S114>/Gain2' */
    rtb_rud = 45.0 * Z1_Sim_B.DataStoreRead.rud;

    /* Gain: '<S114>/Gain4' */
    rtb_flaps = 45.0 * Z1_Sim_B.DataStoreRead.flaps;

    /* Gain: '<S114>/Gain5' */
    rtb_ailOut = 45.0 * Z1_Sim_B.DataStoreRead.ailOut;
  }

  /* SignalConversion generated from: '<S7>/p,q,r ' */
  Z1_Sim_B.TmpSignalConversionAtpqrInport2[0] = Z1_Sim_ConstB.latbit;
  Z1_Sim_B.TmpSignalConversionAtpqrInport2[1] = Z1_Sim_ConstB.longbit;
  Z1_Sim_B.TmpSignalConversionAtpqrInport2[2] = Z1_Sim_ConstB.latbit;

  /* Integrator: '<S7>/p,q,r ' */
  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    zcEvent = rt_ZCFcn(ANY_ZERO_CROSSING,&Z1_Sim_PrevZCX.pqr_Reset_ZCE[0],
                       (Z1_Sim_B.TmpSignalConversionAtpqrInport2[0]));
    if ((zcEvent != NO_ZCEVENT) || (Z1_Sim_B.TmpSignalConversionAtpqrInport2[0]
         != 0.0)) {
      Z1_Sim_X.p[0] = 0.0;
    }

    zcEvent = rt_ZCFcn(ANY_ZERO_CROSSING,&Z1_Sim_PrevZCX.pqr_Reset_ZCE[1],
                       (Z1_Sim_B.TmpSignalConversionAtpqrInport2[1]));
    if ((zcEvent != NO_ZCEVENT) || (Z1_Sim_B.TmpSignalConversionAtpqrInport2[1]
         != 0.0)) {
      Z1_Sim_X.p[1] = 0.0;
    }

    zcEvent = rt_ZCFcn(ANY_ZERO_CROSSING,&Z1_Sim_PrevZCX.pqr_Reset_ZCE[2],
                       (Z1_Sim_B.TmpSignalConversionAtpqrInport2[2]));
    if ((zcEvent != NO_ZCEVENT) || (Z1_Sim_B.TmpSignalConversionAtpqrInport2[2]
         != 0.0)) {
      Z1_Sim_X.p[2] = 0.0;
    }
  }

  /* SignalConversion generated from: '<S85>/q0 q1 q2 q3' */
  Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[0] = Z1_Sim_ConstB.q0;
  Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[1] = Z1_Sim_ConstB.q1;
  Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[2] = Z1_Sim_ConstB.q2;
  Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[3] = Z1_Sim_ConstB.q3;

  /* Integrator: '<S85>/q0 q1 q2 q3' */
  if (Z1_Sim_DW.q0q1q2q3_IWORK != 0) {
    Z1_Sim_X.qr[0] = Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[0];
    Z1_Sim_X.qr[1] = Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[1];
    Z1_Sim_X.qr[2] = Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[2];
    Z1_Sim_X.qr[3] = Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[3];
  }

  /* Sqrt: '<S101>/sqrt' incorporates:
   *  Integrator: '<S85>/q0 q1 q2 q3'
   *  Product: '<S102>/Product'
   *  Product: '<S102>/Product1'
   *  Product: '<S102>/Product2'
   *  Product: '<S102>/Product3'
   *  Sqrt: '<S109>/sqrt'
   *  Sum: '<S102>/Sum'
   */
  rtb_fcn3 = std::sqrt(((Z1_Sim_X.qr[0] * Z1_Sim_X.qr[0] + Z1_Sim_X.qr[1] *
    Z1_Sim_X.qr[1]) + Z1_Sim_X.qr[2] * Z1_Sim_X.qr[2]) + Z1_Sim_X.qr[3] *
                       Z1_Sim_X.qr[3]);

  /* Product: '<S100>/Product' incorporates:
   *  Integrator: '<S85>/q0 q1 q2 q3'
   *  Sqrt: '<S101>/sqrt'
   */
  rtb_referencearea = Z1_Sim_X.qr[0] / rtb_fcn3;

  /* Product: '<S100>/Product1' incorporates:
   *  Integrator: '<S85>/q0 q1 q2 q3'
   *  Sqrt: '<S101>/sqrt'
   */
  rtb_InterpolationUsingPrelook_m = Z1_Sim_X.qr[1] / rtb_fcn3;

  /* Product: '<S100>/Product2' incorporates:
   *  Integrator: '<S85>/q0 q1 q2 q3'
   *  Product: '<S104>/Product2'
   *  Sqrt: '<S101>/sqrt'
   */
  rtb_Saturation_tmp = Z1_Sim_X.qr[2] / rtb_fcn3;

  /* Product: '<S100>/Product3' incorporates:
   *  Integrator: '<S85>/q0 q1 q2 q3'
   *  Product: '<S104>/Product3'
   *  Sqrt: '<S101>/sqrt'
   */
  rtb_f_f_tmp = Z1_Sim_X.qr[3] / rtb_fcn3;

  /* Product: '<S90>/Product3' incorporates:
   *  Product: '<S94>/Product3'
   */
  rtb_Product_c = rtb_referencearea * rtb_referencearea;

  /* Product: '<S90>/Product2' incorporates:
   *  Product: '<S94>/Product2'
   */
  rtb_UnitConversion_i = rtb_InterpolationUsingPrelook_m *
    rtb_InterpolationUsingPrelook_m;

  /* Product: '<S90>/Product1' incorporates:
   *  Product: '<S100>/Product2'
   *  Product: '<S94>/Product1'
   *  Product: '<S98>/Product1'
   */
  rtb_MediumHighAltitudeIntensity = rtb_Saturation_tmp * rtb_Saturation_tmp;

  /* Product: '<S90>/Product' incorporates:
   *  Product: '<S100>/Product3'
   *  Product: '<S94>/Product'
   *  Product: '<S98>/Product'
   */
  rtb_Saturation = rtb_f_f_tmp * rtb_f_f_tmp;

  /* Sum: '<S90>/Sum' incorporates:
   *  Product: '<S90>/Product'
   *  Product: '<S90>/Product1'
   *  Product: '<S90>/Product2'
   *  Product: '<S90>/Product3'
   */
  Z1_Sim_B.VectorConcatenate[0] = ((rtb_Product_c + rtb_UnitConversion_i) -
    rtb_MediumHighAltitudeIntensity) - rtb_Saturation;

  /* Product: '<S93>/Product3' incorporates:
   *  Product: '<S100>/Product3'
   *  Product: '<S91>/Product3'
   */
  rtb_f_n = rtb_f_f_tmp * rtb_referencearea;

  /* Product: '<S93>/Product2' incorporates:
   *  Product: '<S100>/Product2'
   *  Product: '<S91>/Product2'
   */
  rtb_sigma_ugsigma_vg = rtb_InterpolationUsingPrelook_m * rtb_Saturation_tmp;

  /* Gain: '<S93>/Gain' incorporates:
   *  Product: '<S93>/Product2'
   *  Product: '<S93>/Product3'
   *  Sum: '<S93>/Sum'
   */
  Z1_Sim_B.VectorConcatenate[1] = (rtb_sigma_ugsigma_vg - rtb_f_n) * 2.0;

  /* Product: '<S96>/Product2' incorporates:
   *  Product: '<S100>/Product3'
   *  Product: '<S92>/Product2'
   */
  rtb_InterpolationUsingPrelookup = rtb_InterpolationUsingPrelook_m *
    rtb_f_f_tmp;

  /* Product: '<S96>/Product1' incorporates:
   *  Product: '<S100>/Product2'
   *  Product: '<S92>/Product1'
   */
  rtb_InterpolationUsingPreloo_nk = rtb_referencearea * rtb_Saturation_tmp;

  /* Gain: '<S96>/Gain' incorporates:
   *  Product: '<S96>/Product1'
   *  Product: '<S96>/Product2'
   *  Sum: '<S96>/Sum'
   */
  Z1_Sim_B.VectorConcatenate[2] = (rtb_InterpolationUsingPreloo_nk +
    rtb_InterpolationUsingPrelookup) * 2.0;

  /* Gain: '<S91>/Gain' incorporates:
   *  Sum: '<S91>/Sum'
   */
  Z1_Sim_B.VectorConcatenate[3] = (rtb_f_n + rtb_sigma_ugsigma_vg) * 2.0;

  /* Sum: '<S94>/Sum' incorporates:
   *  Sum: '<S98>/Sum'
   */
  rtb_Product_c -= rtb_UnitConversion_i;
  Z1_Sim_B.VectorConcatenate[4] = (rtb_Product_c +
    rtb_MediumHighAltitudeIntensity) - rtb_Saturation;

  /* Product: '<S97>/Product1' incorporates:
   *  Product: '<S95>/Product1'
   */
  rtb_UnitConversion_i = rtb_referencearea * rtb_InterpolationUsingPrelook_m;

  /* Product: '<S97>/Product2' incorporates:
   *  Product: '<S100>/Product2'
   *  Product: '<S100>/Product3'
   *  Product: '<S95>/Product2'
   */
  rtb_f_n = rtb_Saturation_tmp * rtb_f_f_tmp;

  /* Gain: '<S97>/Gain' incorporates:
   *  Product: '<S97>/Product1'
   *  Product: '<S97>/Product2'
   *  Sum: '<S97>/Sum'
   */
  Z1_Sim_B.VectorConcatenate[5] = (rtb_f_n - rtb_UnitConversion_i) * 2.0;

  /* Gain: '<S92>/Gain' incorporates:
   *  Sum: '<S92>/Sum'
   */
  Z1_Sim_B.VectorConcatenate[6] = (rtb_InterpolationUsingPrelookup -
    rtb_InterpolationUsingPreloo_nk) * 2.0;

  /* Gain: '<S95>/Gain' incorporates:
   *  Sum: '<S95>/Sum'
   */
  Z1_Sim_B.VectorConcatenate[7] = (rtb_UnitConversion_i + rtb_f_n) * 2.0;

  /* Sum: '<S98>/Sum' */
  Z1_Sim_B.VectorConcatenate[8] = (rtb_Product_c -
    rtb_MediumHighAltitudeIntensity) + rtb_Saturation;

  /* SignalConversion generated from: '<S7>/ub,vb,wb' */
  Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[0] = Z1_Sim_ConstB.longbit;
  Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[1] = Z1_Sim_ConstB.latbit;
  Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[2] = Z1_Sim_ConstB.longbit;

  /* Integrator: '<S7>/ub,vb,wb' */
  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    zcEvent = rt_ZCFcn(ANY_ZERO_CROSSING,&Z1_Sim_PrevZCX.ubvbwb_Reset_ZCE[0],
                       (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[0]));
    if ((zcEvent != NO_ZCEVENT) || (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[0]
         != 0.0)) {
      Z1_Sim_X.u[0] = 14.2;
    }

    zcEvent = rt_ZCFcn(ANY_ZERO_CROSSING,&Z1_Sim_PrevZCX.ubvbwb_Reset_ZCE[1],
                       (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[1]));
    if ((zcEvent != NO_ZCEVENT) || (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[1]
         != 0.0)) {
      Z1_Sim_X.u[1] = 0.0;
    }

    zcEvent = rt_ZCFcn(ANY_ZERO_CROSSING,&Z1_Sim_PrevZCX.ubvbwb_Reset_ZCE[2],
                       (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[2]));
    if ((zcEvent != NO_ZCEVENT) || (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[2]
         != 0.0)) {
      Z1_Sim_X.u[2] = 0.3674;
    }
  }

  /* Outputs for IfAction SubSystem: '<S118>/If Warning//Error' incorporates:
   *  ActionPort: '<S142>/if'
   */
  for (i = 0; i < 3; i++) {
    /* If: '<S118>/If1' incorporates:
     *  Math: '<S145>/Math Function'
     *  Math: '<S7>/Transpose'
     */
    rtb_VectorConcatenate_m[3 * i] = Z1_Sim_B.VectorConcatenate[i];
    rtb_VectorConcatenate_m[3 * i + 1] = Z1_Sim_B.VectorConcatenate[i + 3];
    rtb_VectorConcatenate_m[3 * i + 2] = Z1_Sim_B.VectorConcatenate[i + 6];
  }

  /* End of Outputs for SubSystem: '<S118>/If Warning//Error' */

  /* Product: '<S79>/Product' incorporates:
   *  Integrator: '<S7>/ub,vb,wb'
   *  Math: '<S7>/Transpose'
   */
  for (i = 0; i < 3; i++) {
    Z1_Sim_B.Product[i] = 0.0;
    Z1_Sim_B.Product[i] += rtb_VectorConcatenate_m[i] * Z1_Sim_X.u[0];
    Z1_Sim_B.Product[i] += rtb_VectorConcatenate_m[i + 3] * Z1_Sim_X.u[1];
    Z1_Sim_B.Product[i] += rtb_VectorConcatenate_m[i + 6] * Z1_Sim_X.u[2];
  }

  /* End of Product: '<S79>/Product' */

  /* Integrator: '<S7>/xe,ye,ze' */
  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    zcEvent = rt_ZCFcn(ANY_ZERO_CROSSING,&Z1_Sim_PrevZCX.xeyeze_Reset_ZCE[0],
                       (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[0]));
    if ((zcEvent != NO_ZCEVENT) || (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[0]
         != 0.0)) {
      Z1_Sim_X.Xe[0] = 0.0;
    }

    zcEvent = rt_ZCFcn(ANY_ZERO_CROSSING,&Z1_Sim_PrevZCX.xeyeze_Reset_ZCE[1],
                       (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[1]));
    if ((zcEvent != NO_ZCEVENT) || (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[1]
         != 0.0)) {
      Z1_Sim_X.Xe[1] = 0.0;
    }

    zcEvent = rt_ZCFcn(ANY_ZERO_CROSSING,&Z1_Sim_PrevZCX.xeyeze_Reset_ZCE[2],
                       (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[2]));
    if ((zcEvent != NO_ZCEVENT) || (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[2]
         != 0.0)) {
      Z1_Sim_X.Xe[2] = -304.8;
    }
  }

  /* Sum: '<S154>/Add' incorporates:
   *  Gain: '<S1>/xyH'
   *  Integrator: '<S7>/xe,ye,ze'
   */
  Z1_Sim_B.Add[0] = 37.684341;
  Z1_Sim_B.Add[1] = -121.436448;
  Z1_Sim_B.Add[2] = -Z1_Sim_X.Xe[2];

  /* S-Function (saeroatmos): '<S157>/S-Function' */
  {
    /* S-Function Block: <S157>/S-Function */
    real_T *temp_table = (real_T *) &Z1_Sim_DW.SFunction_temp_table[0];
    real_T *pres_table = (real_T *) &Z1_Sim_DW.SFunction_pres_table[0];

    /* COESA */
    CalcAtmosCOESA( &Z1_Sim_B.Add[2], &Z1_Sim_B.SFunction_o1,
                   &Z1_Sim_B.SFunction_o3, &Z1_Sim_B.SFunction_o4,
                   &Z1_Sim_B.SFunction_o2, temp_table, pres_table, 1);
  }

  /* Gain: '<S66>/reference area' incorporates:
   *  Gain: '<S13>/1//2rhoV^2'
   *  Product: '<S13>/Product2'
   *  Product: '<S15>/Product'
   *  Product: '<S15>/Product1'
   *  Product: '<S15>/Product2'
   *  Sum: '<S15>/Sum'
   */
  rtb_referencearea = ((Z1_Sim_B.Product[0] * Z1_Sim_B.Product[0] +
                        Z1_Sim_B.Product[1] * Z1_Sim_B.Product[1]) +
                       Z1_Sim_B.Product[2] * Z1_Sim_B.Product[2]) *
    Z1_Sim_B.SFunction_o4 * 0.5 * 4.0;

  /* UnitConversion: '<S210>/Unit Conversion' incorporates:
   *  Gain: '<S1>/xyH'
   *  Integrator: '<S7>/xe,ye,ze'
   */
  /* Unit Conversion - from: m to: ft
     Expression: output = (3.28084*input) + (0) */
  rtb_InterpolationUsingPrelook_m = 3.280839895013123 * -Z1_Sim_X.Xe[2];

  /* Saturate: '<S165>/3ft-->inf' */
  if (rtb_InterpolationUsingPrelook_m <= 3.0) {
    rtb_InterpolationUsingPrelook_m = 3.0;
  }

  /* End of Saturate: '<S165>/3ft-->inf' */

  /* Gain: '<S165>/h//z0' */
  rtb_InterpolationUsingPrelook_m *= 6.666666666666667;

  /* Math: '<S165>/ln(h//z0)'
   *
   * About '<S165>/ln(h//z0)':
   *  Operator: log
   */
  rtb_InterpolationUsingPrelook_m = std::log(rtb_InterpolationUsingPrelook_m);

  /* Product: '<S165>/Product' */
  rtb_Product_c = rtb_InterpolationUsingPrelook_m / 4.8928522584398726;

  /* Product: '<S165>/Product1' */
  rtb_Sum2_m[0] = rtb_Product_c * -0.0;
  rtb_Sum2_m[1] = rtb_Product_c * -0.0;
  rtb_Sum2_m[2] = rtb_Product_c * -0.0;

  /* UnitConversion: '<S173>/Unit Conversion' incorporates:
   *  Gain: '<S1>/xyH'
   *  Integrator: '<S7>/xe,ye,ze'
   */
  /* Unit Conversion - from: m to: ft
     Expression: output = (3.28084*input) + (0) */
  rtb_Product_c = 3.280839895013123 * -Z1_Sim_X.Xe[2];

  /* DotProduct: '<S156>/Dot Product' */
  rtb_Product_d_idx_2 = 0.0;
  for (i = 0; i < 3; i++) {
    /* DotProduct: '<S156>/Dot Product' */
    rtb_Product_d_idx_2 += Z1_Sim_B.Product[i] * Z1_Sim_B.Product[i];

    /* Product: '<S165>/Transform from Inertial to Body axes' */
    rtb_Sum_gb[i] = Z1_Sim_B.VectorConcatenate[i + 6] * rtb_Sum2_m[2] +
      (Z1_Sim_B.VectorConcatenate[i + 3] * rtb_Sum2_m[1] +
       Z1_Sim_B.VectorConcatenate[i] * rtb_Sum2_m[0]);
  }

  /* Sqrt: '<S156>/Sqrt' incorporates:
   *  DotProduct: '<S156>/Dot Product'
   */
  Z1_Sim_B.Sqrt = std::sqrt(rtb_Product_d_idx_2);

  /* UnitConversion: '<S179>/Unit Conversion' */
  /* Unit Conversion - from: m/s to: ft/s
     Expression: output = (3.28084*input) + (0) */
  rtb_UnitConversion_i = 3.280839895013123 * Z1_Sim_B.Sqrt;

  /* Saturate: '<S206>/Limit Function 10ft to 1000ft' incorporates:
   *  Saturate: '<S189>/Limit Height h<1000ft'
   */
  if (rtb_Product_c > 1000.0) {
    rtb_InterpolationUsingPrelook_m = 1000.0;
    rtb_Saturation = 1000.0;
  } else {
    if (rtb_Product_c < 10.0) {
      rtb_InterpolationUsingPrelook_m = 10.0;
    } else {
      rtb_InterpolationUsingPrelook_m = rtb_Product_c;
    }

    if (rtb_Product_c < 0.0) {
      rtb_Saturation = 0.0;
    } else {
      rtb_Saturation = rtb_Product_c;
    }
  }

  /* End of Saturate: '<S206>/Limit Function 10ft to 1000ft' */

  /* Fcn: '<S206>/Low Altitude Scale Length' */
  rtb_f_n = rtb_InterpolationUsingPrelook_m / rt_powd_snf(0.000823 *
    rtb_InterpolationUsingPrelook_m + 0.177, 1.2);

  /* Product: '<S189>/sigma_ug, sigma_vg' incorporates:
   *  Fcn: '<S189>/Low Altitude Intensity'
   */
  rtb_sigma_ugsigma_vg = 1.0 / rt_powd_snf(0.000823 * rtb_Saturation + 0.177,
    0.4) * 1.6404199475065615;

  /* Interpolation_n-D: '<S188>/Medium//High Altitude Intensity' incorporates:
   *  PreLookup: '<S188>/PreLook-Up Index Search  (altitude)'
   */
  bpIndex[0] = plook_bincpa(rtb_Product_c,
    Z1_Sim_ConstP.PreLookUpIndexSearchaltitude_Br, 11U, &rtb_Saturation,
    &Z1_Sim_DW.PreLookUpIndexSearchaltitude_DW);
  frac[0] = rtb_Saturation;
  frac[1] = 0.0;
  bpIndex[1] = 2U;
  rtb_MediumHighAltitudeIntensity = intrp2d_la(bpIndex, frac,
    Z1_Sim_ConstP.MediumHighAltitudeIntensity_Tab, 12U,
    Z1_Sim_ConstP.MediumHighAltitudeIntensity_max);
  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[2] == 0) {
    /* Product: '<S181>/Product' incorporates:
     *  RandomNumber: '<S181>/White Noise'
     */
    Z1_Sim_B.Product_j[0] = 5.6049912163979281 * Z1_Sim_DW.NextOutput[0];
    Z1_Sim_B.Product_j[1] = 5.6049912163979281 * Z1_Sim_DW.NextOutput[1];
    Z1_Sim_B.Product_j[2] = 5.6049912163979281 * Z1_Sim_DW.NextOutput[2];
    Z1_Sim_B.Product_j[3] = 5.6049912163979281 * Z1_Sim_DW.NextOutput[3];
  }

  /* Outputs for Enabled SubSystem: '<S172>/Hvgw(s)' incorporates:
   *  EnablePort: '<S186>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S172>/Hugw(s)' incorporates:
   *  EnablePort: '<S185>/Enable'
   */
  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
    if (rtmIsMajorTimeStep((&Z1_Sim_M)) && Z1_Sim_DW.Hugws_MODE) {
      /* Disable for Outport: '<S185>/ugw' */
      Z1_Sim_B.w1_p[0] = 0.0;
      Z1_Sim_B.w1_p[1] = 0.0;
      Z1_Sim_DW.Hugws_MODE = false;
    }

    if (rtmIsMajorTimeStep((&Z1_Sim_M)) && Z1_Sim_DW.Hvgws_MODE) {
      /* Disable for Outport: '<S186>/vgw' */
      Z1_Sim_B.w1[0] = 0.0;
      Z1_Sim_B.w1[1] = 0.0;
      Z1_Sim_DW.Hvgws_MODE = false;
    }
  }

  /* End of Outputs for SubSystem: '<S172>/Hvgw(s)' */
  if (Z1_Sim_DW.Hugws_MODE) {
    /* Product: '<S185>/Lug//V' */
    rtb_Saturation = rtb_f_n / rtb_UnitConversion_i;

    /* Product: '<S185>/w' incorporates:
     *  Gain: '<S185>/(2//pi)'
     *  Integrator: '<S185>/ug_p'
     *  Product: '<S185>/Lug//V'
     *  Product: '<S185>/Lug//V1'
     *  Sqrt: '<S185>/sqrt'
     *  Sum: '<S185>/Sum'
     */
    Z1_Sim_B.w_e[0] = (std::sqrt(rtb_Saturation * 0.63661977236758138) *
                       Z1_Sim_B.Product_j[0] - Z1_Sim_X.ug_p_CSTATE[0]) /
      rtb_Saturation;

    /* Product: '<S185>/Lug//V' */
    rtb_Saturation = Z1_Sim_ConstB.UnitConversion_cm / rtb_UnitConversion_i;

    /* Product: '<S185>/w' incorporates:
     *  Gain: '<S185>/(2//pi)'
     *  Integrator: '<S185>/ug_p'
     *  Product: '<S185>/Lug//V'
     *  Product: '<S185>/Lug//V1'
     *  Sqrt: '<S185>/sqrt'
     *  Sum: '<S185>/Sum'
     */
    Z1_Sim_B.w_e[1] = (std::sqrt(rtb_Saturation * 0.63661977236758138) *
                       Z1_Sim_B.Product_j[0] - Z1_Sim_X.ug_p_CSTATE[1]) /
      rtb_Saturation;

    /* Product: '<S185>/w1' incorporates:
     *  Integrator: '<S185>/ug_p'
     */
    Z1_Sim_B.w1_p[0] = Z1_Sim_X.ug_p_CSTATE[0] * rtb_sigma_ugsigma_vg;
    Z1_Sim_B.w1_p[1] = Z1_Sim_X.ug_p_CSTATE[1] * rtb_MediumHighAltitudeIntensity;
  }

  /* End of Outputs for SubSystem: '<S172>/Hugw(s)' */

  /* Outputs for Enabled SubSystem: '<S172>/Hvgw(s)' incorporates:
   *  EnablePort: '<S186>/Enable'
   */
  if (Z1_Sim_DW.Hvgws_MODE) {
    /* Product: '<S186>/Lvg//V' incorporates:
     *  Gain: '<S178>/Lv'
     */
    frac[0] = rtb_f_n / rtb_UnitConversion_i;
    frac[1] = Z1_Sim_ConstB.UnitConversion_cm / rtb_UnitConversion_i;

    /* Product: '<S186>/w' incorporates:
     *  Gain: '<S186>/(1//pi)'
     *  Integrator: '<S186>/vg_p1'
     *  Product: '<S186>/Lug//V1'
     *  Sqrt: '<S186>/sqrt'
     *  Sum: '<S186>/Sum'
     */
    Z1_Sim_B.w_h[0] = (std::sqrt(0.31830988618379069 * frac[0]) *
                       Z1_Sim_B.Product_j[1] - Z1_Sim_X.vg_p1_CSTATE[0]) / frac
      [0];

    /* Product: '<S186>/w ' incorporates:
     *  Gain: '<S186>/(1//pi)'
     *  Gain: '<S186>/sqrt(3)'
     *  Integrator: '<S186>/vg_p1'
     *  Integrator: '<S186>/vgw_p2'
     *  Product: '<S186>/Lvg//V '
     *  Sum: '<S186>/Sum1'
     */
    Z1_Sim_B.w_j[0] = (Z1_Sim_B.w_h[0] * frac[0] * 1.7320508075688772 +
                       (Z1_Sim_X.vg_p1_CSTATE[0] - Z1_Sim_X.vgw_p2_CSTATE[0])) /
      frac[0];

    /* Product: '<S186>/w' incorporates:
     *  Gain: '<S186>/(1//pi)'
     *  Integrator: '<S186>/vg_p1'
     *  Product: '<S186>/Lug//V1'
     *  Sqrt: '<S186>/sqrt'
     *  Sum: '<S186>/Sum'
     */
    Z1_Sim_B.w_h[1] = (std::sqrt(0.31830988618379069 * frac[1]) *
                       Z1_Sim_B.Product_j[1] - Z1_Sim_X.vg_p1_CSTATE[1]) / frac
      [1];

    /* Product: '<S186>/w ' incorporates:
     *  Gain: '<S186>/(1//pi)'
     *  Gain: '<S186>/sqrt(3)'
     *  Integrator: '<S186>/vg_p1'
     *  Integrator: '<S186>/vgw_p2'
     *  Product: '<S186>/Lvg//V '
     *  Sum: '<S186>/Sum1'
     */
    Z1_Sim_B.w_j[1] = (Z1_Sim_B.w_h[1] * frac[1] * 1.7320508075688772 +
                       (Z1_Sim_X.vg_p1_CSTATE[1] - Z1_Sim_X.vgw_p2_CSTATE[1])) /
      frac[1];

    /* Product: '<S186>/w 1' incorporates:
     *  Integrator: '<S186>/vgw_p2'
     */
    Z1_Sim_B.w1[0] = rtb_sigma_ugsigma_vg * Z1_Sim_X.vgw_p2_CSTATE[0];
    Z1_Sim_B.w1[1] = rtb_MediumHighAltitudeIntensity * Z1_Sim_X.vgw_p2_CSTATE[1];
  }

  /* End of Outputs for SubSystem: '<S172>/Hvgw(s)' */

  /* Gain: '<S178>/Lw' */
  frac[0] = rtb_InterpolationUsingPrelook_m;

  /* Outputs for Enabled SubSystem: '<S172>/Hwgw(s)' incorporates:
   *  EnablePort: '<S187>/Enable'
   */
  if ((rtmIsMajorTimeStep((&Z1_Sim_M)) &&
       (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) && rtmIsMajorTimeStep
      ((&Z1_Sim_M)) && Z1_Sim_DW.Hwgws_MODE) {
    /* Disable for Outport: '<S187>/wgw' */
    Z1_Sim_B.LwgV1[0] = 0.0;
    Z1_Sim_B.LwgV1[1] = 0.0;
    Z1_Sim_DW.Hwgws_MODE = false;
  }

  if (Z1_Sim_DW.Hwgws_MODE) {
    /* Product: '<S187>/Lwg//V 1' incorporates:
     *  Integrator: '<S187>/wg_p2'
     */
    Z1_Sim_B.LwgV1[0] = 1.6404199475065615 * Z1_Sim_X.wg_p2_CSTATE[0];
    Z1_Sim_B.LwgV1[1] = rtb_MediumHighAltitudeIntensity * Z1_Sim_X.wg_p2_CSTATE
      [1];

    /* Product: '<S187>/Lwg//V' incorporates:
     *  Gain: '<S178>/Lw'
     */
    rtb_InterpolationUsingPrelook_m /= rtb_UnitConversion_i;

    /* Product: '<S187>/w' incorporates:
     *  Gain: '<S187>/1//pi'
     *  Integrator: '<S187>/wg_p1'
     *  Product: '<S187>/Lug//V1'
     *  Sqrt: '<S187>/sqrt1'
     *  Sum: '<S187>/Sum'
     */
    Z1_Sim_B.w[0] = (std::sqrt(0.31830988618379069 *
      rtb_InterpolationUsingPrelook_m) * Z1_Sim_B.Product_j[2] -
                     Z1_Sim_X.wg_p1_CSTATE[0]) / rtb_InterpolationUsingPrelook_m;

    /* Product: '<S187>/w ' incorporates:
     *  Integrator: '<S187>/wg_p1'
     *  Integrator: '<S187>/wg_p2'
     *  Product: '<S187>/Lwg//V '
     *  Sum: '<S187>/Sum1'
     */
    Z1_Sim_B.w_k[0] = (Z1_Sim_B.w[0] * 1.7320508075688772 *
                       rtb_InterpolationUsingPrelook_m + (Z1_Sim_X.wg_p1_CSTATE
      [0] - Z1_Sim_X.wg_p2_CSTATE[0])) / rtb_InterpolationUsingPrelook_m;

    /* Product: '<S187>/Lwg//V' incorporates:
     *  Gain: '<S178>/Lw'
     */
    rtb_InterpolationUsingPrelook_m = Z1_Sim_ConstB.UnitConversion_cm /
      rtb_UnitConversion_i;

    /* Product: '<S187>/w' incorporates:
     *  Gain: '<S187>/1//pi'
     *  Integrator: '<S187>/wg_p1'
     *  Product: '<S187>/Lug//V1'
     *  Sqrt: '<S187>/sqrt1'
     *  Sum: '<S187>/Sum'
     */
    Z1_Sim_B.w[1] = (std::sqrt(0.31830988618379069 *
      rtb_InterpolationUsingPrelook_m) * Z1_Sim_B.Product_j[2] -
                     Z1_Sim_X.wg_p1_CSTATE[1]) / rtb_InterpolationUsingPrelook_m;

    /* Product: '<S187>/w ' incorporates:
     *  Integrator: '<S187>/wg_p1'
     *  Integrator: '<S187>/wg_p2'
     *  Product: '<S187>/Lwg//V '
     *  Sum: '<S187>/Sum1'
     */
    Z1_Sim_B.w_k[1] = (Z1_Sim_B.w[1] * 1.7320508075688772 *
                       rtb_InterpolationUsingPrelook_m + (Z1_Sim_X.wg_p1_CSTATE
      [1] - Z1_Sim_X.wg_p2_CSTATE[1])) / rtb_InterpolationUsingPrelook_m;
  }

  /* End of Outputs for SubSystem: '<S172>/Hwgw(s)' */

  /* If: '<S177>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    if (rtb_Product_c <= 1000.0) {
      rtAction = 0;
    } else if (rtb_Product_c >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    Z1_Sim_DW.ifHeightMaxlowaltitudeelseifHei = rtAction;
  } else {
    rtAction = Z1_Sim_DW.ifHeightMaxlowaltitudeelseifHei;
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S177>/Low altitude  velocities' incorporates:
     *  ActionPort: '<S199>/Action Port'
     */
    /* Sum: '<S205>/Sum' incorporates:
     *  Product: '<S205>/Product1'
     *  Product: '<S205>/Product2'
     */
    rtb_Product1_pxv[0] = Z1_Sim_B.w1_p[0] - 0.0 * Z1_Sim_B.w1[0];

    /* Sum: '<S205>/Sum1' incorporates:
     *  Product: '<S205>/Product1'
     *  Product: '<S205>/Product2'
     */
    rtb_Product1_pxv[1] = 0.0 * Z1_Sim_B.w1_p[0] + Z1_Sim_B.w1[0];

    /* Reshape: '<S204>/Reshape1' incorporates:
     *  Product: '<S204>/Product'
     *  SignalConversion generated from: '<S204>/Vector Concatenate'
     */
    for (i = 0; i < 3; i++) {
      rtb_Sum2_m[i] = Z1_Sim_B.VectorConcatenate[i + 6] * Z1_Sim_B.LwgV1[0] +
        (Z1_Sim_B.VectorConcatenate[i + 3] * rtb_Product1_pxv[1] +
         Z1_Sim_B.VectorConcatenate[i] * rtb_Product1_pxv[0]);
    }

    /* End of Reshape: '<S204>/Reshape1' */
    /* End of Outputs for SubSystem: '<S177>/Low altitude  velocities' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S177>/Medium//High  altitude velocities' incorporates:
     *  ActionPort: '<S200>/Action Port'
     */
    /* Gain: '<S200>/Gain' */
    rtb_Sum2_m[0] = Z1_Sim_B.w1_p[1];
    rtb_Sum2_m[1] = Z1_Sim_B.w1[1];
    rtb_Sum2_m[2] = Z1_Sim_B.LwgV1[1];

    /* End of Outputs for SubSystem: '<S177>/Medium//High  altitude velocities' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S177>/Interpolate  velocities' incorporates:
     *  ActionPort: '<S198>/Action Port'
     */
    /* Sum: '<S203>/Sum' incorporates:
     *  Product: '<S203>/Product1'
     *  Product: '<S203>/Product2'
     */
    rtb_Sum2_m[0] = Z1_Sim_B.w1_p[0] - 0.0 * Z1_Sim_B.w1[0];

    /* Sum: '<S203>/Sum1' incorporates:
     *  Product: '<S203>/Product1'
     *  Product: '<S203>/Product2'
     */
    rtb_Sum2_m[1] = 0.0 * Z1_Sim_B.w1_p[0] + Z1_Sim_B.w1[0];

    /* Product: '<S202>/Product' incorporates:
     *  SignalConversion generated from: '<S202>/Vector Concatenate'
     */
    for (i = 0; i < 3; i++) {
      rtb_Product1_pxv[i] = Z1_Sim_B.VectorConcatenate[i + 6] * Z1_Sim_B.LwgV1[0]
        + (Z1_Sim_B.VectorConcatenate[i + 3] * rtb_Sum2_m[1] +
           Z1_Sim_B.VectorConcatenate[i] * rtb_Sum2_m[0]);
    }

    /* End of Product: '<S202>/Product' */

    /* Sum: '<S198>/Sum3' incorporates:
     *  Constant: '<S198>/max_height_low'
     *  Product: '<S198>/Product1'
     *  Sum: '<S198>/Sum1'
     *  Sum: '<S198>/Sum2'
     */
    rtb_Sum2_m[0] = (Z1_Sim_B.w1_p[1] - rtb_Product1_pxv[0]) * (rtb_Product_c -
      1000.0) / 1000.0 + rtb_Product1_pxv[0];
    rtb_Sum2_m[1] = (Z1_Sim_B.w1[1] - rtb_Product1_pxv[1]) * (rtb_Product_c -
      1000.0) / 1000.0 + rtb_Product1_pxv[1];
    rtb_Sum2_m[2] = (Z1_Sim_B.LwgV1[1] - rtb_Product1_pxv[2]) * (rtb_Product_c -
      1000.0) / 1000.0 + rtb_Product1_pxv[2];

    /* End of Outputs for SubSystem: '<S177>/Interpolate  velocities' */
    break;
  }

  /* End of If: '<S177>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */

  /* UnitConversion: '<S164>/Unit Conversion' */
  /* Unit Conversion - from: ft/s to: m/s
     Expression: output = (0.3048*input) + (0) */
  rtb_Sum2_m[0] *= 0.3048;
  rtb_Sum2_m[1] *= 0.3048;

  /* Logic: '<S163>/Logical Operator2' incorporates:
   *  Constant: '<S163>/Constant'
   */
  Z1_Sim_B.LogicalOperator2 = false;
  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
    /* SignalConversion generated from: '<S166>/Enable' */
    Z1_Sim_B.HiddenBuf_InsertedFor_Distancei = Z1_Sim_B.LogicalOperator2;

    /* Outputs for Enabled SubSystem: '<S163>/Distance into gust (x)' incorporates:
     *  EnablePort: '<S166>/Enable'
     */
    if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
      if (Z1_Sim_B.HiddenBuf_InsertedFor_Distancei) {
        if (!Z1_Sim_DW.Distanceintogustx_MODE) {
          /* InitializeConditions for Integrator: '<S166>/Distance into Gust (x) (Limited to gust length d)' */
          Z1_Sim_X.DistanceintoGustxLimitedtogus_n = 0.0;
          Z1_Sim_DW.Distanceintogustx_MODE = true;
        }
      } else {
        Z1_Sim_DW.Distanceintogustx_MODE = false;
      }
    }

    /* End of Outputs for SubSystem: '<S163>/Distance into gust (x)' */
  }

  /* Outputs for Enabled SubSystem: '<S163>/Distance into gust (x)' incorporates:
   *  EnablePort: '<S166>/Enable'
   */
  if (Z1_Sim_DW.Distanceintogustx_MODE) {
    /* Integrator: '<S166>/Distance into Gust (x) (Limited to gust length d)' */
    /* Limited  Integrator  */
    if (Z1_Sim_X.DistanceintoGustxLimitedtogus_n >= 120.0) {
      Z1_Sim_X.DistanceintoGustxLimitedtogus_n = 120.0;
    } else {
      if (Z1_Sim_X.DistanceintoGustxLimitedtogus_n <= 0.0) {
        Z1_Sim_X.DistanceintoGustxLimitedtogus_n = 0.0;
      }
    }

    Z1_Sim_B.DistanceintoGustxLimitedtogustl =
      Z1_Sim_X.DistanceintoGustxLimitedtogus_n;

    /* End of Integrator: '<S166>/Distance into Gust (x) (Limited to gust length d)' */
  }

  /* End of Outputs for SubSystem: '<S163>/Distance into gust (x)' */

  /* Logic: '<S163>/Logical Operator1' incorporates:
   *  Constant: '<S163>/Constant1'
   */
  Z1_Sim_B.LogicalOperator1 = false;
  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
    /* SignalConversion generated from: '<S167>/Enable' */
    Z1_Sim_B.HiddenBuf_InsertedFor_Distanc_m = Z1_Sim_B.LogicalOperator1;
  }

  /* Outputs for Enabled SubSystem: '<S163>/Distance into gust (y)' */
  Z1_Sim_Distanceintogusty(Z1_Sim_B.HiddenBuf_InsertedFor_Distanc_m,
    &Z1_Sim_B.Distanceintogusty, &Z1_Sim_DW.Distanceintogusty,
    &Z1_Sim_X.Distanceintogusty, 120.0);

  /* End of Outputs for SubSystem: '<S163>/Distance into gust (y)' */

  /* Logic: '<S163>/Logical Operator3' incorporates:
   *  Constant: '<S163>/Constant2'
   */
  Z1_Sim_B.LogicalOperator3 = false;
  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
    /* SignalConversion generated from: '<S168>/Enable' */
    Z1_Sim_B.HiddenBuf_InsertedFor_Distanc_l = Z1_Sim_B.LogicalOperator3;
  }

  /* Outputs for Enabled SubSystem: '<S163>/Distance into gust (z)' */
  Z1_Sim_Distanceintogusty(Z1_Sim_B.HiddenBuf_InsertedFor_Distanc_l,
    &Z1_Sim_B.Distanceintogustz, &Z1_Sim_DW.Distanceintogustz,
    &Z1_Sim_X.Distanceintogustz, 80.0);

  /* End of Outputs for SubSystem: '<S163>/Distance into gust (z)' */

  /* Sum: '<S9>/Sum4' incorporates:
   *  Constant: '<S163>/2'
   *  Gain: '<S163>/Gust magnitude//2.0'
   *  Gain: '<S163>/pi//Gust length'
   *  Integrator: '<S7>/ub,vb,wb'
   *  Sum: '<S156>/Sum'
   *  Sum: '<S163>/Sum1'
   *  Trigonometry: '<S163>/cos(pi*x//dm)'
   *  UnitConversion: '<S164>/Unit Conversion'
   */
  rtb_Sum_gb[0] = Z1_Sim_X.u[0] - ((1.0 - std::cos(0.026179938779914941 *
    Z1_Sim_B.DistanceintoGustxLimitedtogustl)) * 1.75 + (rtb_Sum_gb[0] +
    rtb_Sum2_m[0]));
  rtb_Sum_gb[1] = Z1_Sim_X.u[1] - ((1.0 - std::cos(0.026179938779914941 *
    Z1_Sim_B.Distanceintogusty.DistanceintoGustxLimitedtogustl)) * 1.75 +
    (rtb_Sum_gb[1] + rtb_Sum2_m[1]));
  rtb_Product_d_idx_2 = Z1_Sim_X.u[2] - ((1.0 - std::cos(0.039269908169872414 *
    Z1_Sim_B.Distanceintogustz.DistanceintoGustxLimitedtogustl)) * 1.5 +
    (rtb_Sum_gb[2] + 0.3048 * rtb_Sum2_m[2]));
  rtb_Sum_gb[2] = rtb_Product_d_idx_2;

  /* Gain: '<S54>/Gain' incorporates:
   *  Trigonometry: '<S14>/Incidence'
   */
  rtb_InterpolationUsingPrelook_m = 57.295779513082323 * rt_atan2d_snf
    (rtb_Product_d_idx_2, rtb_Sum_gb[0]);

  /* PreLookup generated from: '<S22>/Prelookup' */
  rtb_k_b = plook_binc(rtb_InterpolationUsingPrelook_m, Z1_Sim_ConstP.pooled19,
                       30U, &rtb_InterpolationUsingPrelook_m);

  /* Gain: '<S53>/Gain1' incorporates:
   *  Interpolation_n-D generated from: '<S22>/Interpolation Using Prelookup'
   */
  rtb_f_n = 0.017453292519943295 * intrp1d_l(rtb_k_b,
    rtb_InterpolationUsingPrelook_m, Z1_Sim_ConstP.pooled19);

  /* Trigonometry: '<S52>/sincos' */
  rtb_Saturation = std::cos(rtb_f_n);
  rtb_f_n = std::sin(rtb_f_n);

  /* Product: '<S56>/u(3)*u(4)' */
  rtb_VectorConcatenate_m[0] = rtb_Saturation;

  /* UnaryMinus: '<S59>/Unary Minus' incorporates:
   *  Product: '<S59>/u(2)*u(3)'
   */
  rtb_VectorConcatenate_m[1] = -(0.0 * rtb_Saturation);

  /* UnaryMinus: '<S62>/Unary Minus' */
  rtb_VectorConcatenate_m[2] = -rtb_f_n;

  /* SignalConversion generated from: '<S65>/Vector Concatenate' */
  rtb_VectorConcatenate_m[3] = 0.0;

  /* SignalConversion generated from: '<S65>/Vector Concatenate' */
  rtb_VectorConcatenate_m[4] = 1.0;

  /* SignalConversion generated from: '<S65>/Vector Concatenate' incorporates:
   *  Constant: '<S63>/Constant'
   */
  rtb_VectorConcatenate_m[5] = 0.0;

  /* Product: '<S58>/u(1)*u(4)' */
  rtb_VectorConcatenate_m[6] = rtb_f_n;

  /* UnaryMinus: '<S61>/Unary Minus' incorporates:
   *  Product: '<S61>/u(1)*u(2)'
   */
  rtb_VectorConcatenate_m[7] = -(rtb_f_n * 0.0);

  /* SignalConversion generated from: '<S65>/Vector Concatenate' */
  rtb_VectorConcatenate_m[8] = rtb_Saturation;

  /* Math: '<S22>/Stab to body' */
  for (i = 0; i < 3; i++) {
    rtb_DCM_bs[3 * i] = rtb_VectorConcatenate_m[i];
    rtb_DCM_bs[3 * i + 1] = rtb_VectorConcatenate_m[i + 3];
    rtb_DCM_bs[3 * i + 2] = rtb_VectorConcatenate_m[i + 6];
  }

  /* End of Math: '<S22>/Stab to body' */

  /* Sqrt: '<S14>/Airspeed' incorporates:
   *  Product: '<S18>/Product'
   *  Product: '<S18>/Product1'
   *  Product: '<S18>/Product2'
   *  Sum: '<S18>/Sum'
   */
  rtb_Saturation = std::sqrt((rtb_Sum_gb[0] * rtb_Sum_gb[0] + rtb_Sum_gb[1] *
    rtb_Sum_gb[1]) + rtb_Product_d_idx_2 * rtb_Product_d_idx_2);

  /* Saturate: '<S14>/Saturation' */
  if (rtb_Saturation <= 0.1) {
    rtb_f_n = 0.1;
  } else {
    rtb_f_n = rtb_Saturation;
  }

  /* End of Saturate: '<S14>/Saturation' */

  /* Product: '<S14>/Product' */
  rtb_f_n = rtb_Sum_gb[1] / rtb_f_n;

  /* Trigonometry: '<S14>/Sideslip' */
  if (rtb_f_n > 1.0) {
    rtb_f_n = 1.0;
  } else {
    if (rtb_f_n < -1.0) {
      rtb_f_n = -1.0;
    }
  }

  /* Gain: '<S55>/Gain' incorporates:
   *  Trigonometry: '<S14>/Sideslip'
   */
  rtb_sigma_ugsigma_vg = 57.295779513082323 * std::asin(rtb_f_n);

  /* PreLookup generated from: '<S22>/Prelookup1' */
  rtb_k = plook_binc(rtb_sigma_ugsigma_vg,
                     Z1_Sim_ConstP.Prelookup1_1_BreakpointsData, 30U,
                     &rtb_sigma_ugsigma_vg);

  /* Saturate: '<S22>/Saturation' */
  if (rtb_Saturation > 50.0) {
    rtb_f_n = 50.0;
  } else if (rtb_Saturation < 0.0) {
    rtb_f_n = 0.0;
  } else {
    rtb_f_n = rtb_Saturation;
  }

  /* End of Saturate: '<S22>/Saturation' */

  /* PreLookup generated from: '<S22>/Prelookup2' */
  rtb_k_e = plook_binx(rtb_f_n, Z1_Sim_ConstP.Prelookup2_1_BreakpointsData, 5U,
                       &rtb_f_n);

  /* Interpolation_n-D generated from: '<S45>/Interpolation Using Prelookup' */
  frac_0[0] = rtb_InterpolationUsingPrelook_m;
  frac_0[1] = rtb_sigma_ugsigma_vg;
  frac_0[2] = rtb_f_n;
  bpIndex_0[0] = rtb_k_b;
  bpIndex_0[1] = rtb_k;
  bpIndex_0[2] = rtb_k_e;
  rtb_InterpolationUsingPrelookup = intrp3d_l(bpIndex_0, frac_0,
    Z1_Sim_ConstP.InterpolationUsingPrelookup_1_T, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S45>/Interpolation Using Prelookup1' */
  frac_1[0] = rtb_InterpolationUsingPrelook_m;
  frac_1[1] = rtb_sigma_ugsigma_vg;
  frac_1[2] = rtb_f_n;
  bpIndex_1[0] = rtb_k_b;
  bpIndex_1[1] = rtb_k;
  bpIndex_1[2] = rtb_k_e;
  rtb_InterpolationUsingPreloo_nk = intrp3d_l(bpIndex_1, frac_1,
    Z1_Sim_ConstP.InterpolationUsingPrelookup1_1_, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S45>/Interpolation Using Prelookup2' */
  frac_2[0] = rtb_InterpolationUsingPrelook_m;
  frac_2[1] = rtb_sigma_ugsigma_vg;
  frac_2[2] = rtb_f_n;
  bpIndex_2[0] = rtb_k_b;
  bpIndex_2[1] = rtb_k;
  bpIndex_2[2] = rtb_k_e;
  rtb_CLtoCz = intrp3d_l(bpIndex_2, frac_2,
    Z1_Sim_ConstP.InterpolationUsingPrelookup2_1_, Z1_Sim_ConstP.pooled24);

  /* Gain: '<S45>/CL to Cz' */
  rtb_CLtoCz = -rtb_CLtoCz;

  /* Interpolation_n-D generated from: '<S45>/Interpolation Using Prelookup3' */
  frac_3[0] = rtb_InterpolationUsingPrelook_m;
  frac_3[1] = rtb_sigma_ugsigma_vg;
  frac_3[2] = rtb_f_n;
  bpIndex_3[0] = rtb_k_b;
  bpIndex_3[1] = rtb_k;
  bpIndex_3[2] = rtb_k_e;
  rtb_InterpolationUsingPrelook_a = intrp3d_l(bpIndex_3, frac_3,
    Z1_Sim_ConstP.InterpolationUsingPrelookup3_1_, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S45>/Interpolation Using Prelookup4' */
  frac_4[0] = rtb_InterpolationUsingPrelook_m;
  frac_4[1] = rtb_sigma_ugsigma_vg;
  frac_4[2] = rtb_f_n;
  bpIndex_4[0] = rtb_k_b;
  bpIndex_4[1] = rtb_k;
  bpIndex_4[2] = rtb_k_e;
  rtb_InterpolationUsingPrelook_p = intrp3d_l(bpIndex_4, frac_4,
    Z1_Sim_ConstP.InterpolationUsingPrelookup4_1_, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S45>/Interpolation Using Prelookup5' */
  frac_5[0] = rtb_InterpolationUsingPrelook_m;
  frac_5[1] = rtb_sigma_ugsigma_vg;
  frac_5[2] = rtb_f_n;
  bpIndex_5[0] = rtb_k_b;
  bpIndex_5[1] = rtb_k;
  bpIndex_5[2] = rtb_k_e;
  rtb_InterpolationUsingPrelook_h = intrp3d_l(bpIndex_5, frac_5,
    Z1_Sim_ConstP.InterpolationUsingPrelookup5_1_, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S36>/Interpolation Using Prelookup' */
  frac_6[0] = rtb_InterpolationUsingPrelook_m;
  frac_6[1] = rtb_sigma_ugsigma_vg;
  frac_6[2] = rtb_f_n;
  bpIndex_6[0] = rtb_k_b;
  bpIndex_6[1] = rtb_k;
  bpIndex_6[2] = rtb_k_e;
  rtb_fcn2 = intrp3d_l(bpIndex_6, frac_6,
                       Z1_Sim_ConstP.InterpolationUsingPrelookup_1_a,
                       Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S36>/Interpolation Using Prelookup1' */
  frac_7[0] = rtb_InterpolationUsingPrelook_m;
  frac_7[1] = rtb_sigma_ugsigma_vg;
  frac_7[2] = rtb_f_n;
  bpIndex_7[0] = rtb_k_b;
  bpIndex_7[1] = rtb_k;
  bpIndex_7[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_7, frac_7,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1__b,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S36>/Interpolation Using Prelookup2' */
  frac_8[0] = rtb_InterpolationUsingPrelook_m;
  frac_8[1] = rtb_sigma_ugsigma_vg;
  frac_8[2] = rtb_f_n;
  bpIndex_8[0] = rtb_k_b;
  bpIndex_8[1] = rtb_k;
  bpIndex_8[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_8, frac_8,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2__e,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S36>/CL to Cz' */
  rtb_q2dot = -rtb_q2dot;

  /* Interpolation_n-D generated from: '<S36>/Interpolation Using Prelookup3' */
  frac_9[0] = rtb_InterpolationUsingPrelook_m;
  frac_9[1] = rtb_sigma_ugsigma_vg;
  frac_9[2] = rtb_f_n;
  bpIndex_9[0] = rtb_k_b;
  bpIndex_9[1] = rtb_k;
  bpIndex_9[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_9, frac_9,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3__k,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S36>/Interpolation Using Prelookup4' */
  frac_a[0] = rtb_InterpolationUsingPrelook_m;
  frac_a[1] = rtb_sigma_ugsigma_vg;
  frac_a[2] = rtb_f_n;
  bpIndex_a[0] = rtb_k_b;
  bpIndex_a[1] = rtb_k;
  bpIndex_a[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_a, frac_a,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4__j,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S36>/Interpolation Using Prelookup5' */
  frac_b[0] = rtb_InterpolationUsingPrelook_m;
  frac_b[1] = rtb_sigma_ugsigma_vg;
  frac_b[2] = rtb_f_n;
  bpIndex_b[0] = rtb_k_b;
  bpIndex_b[1] = rtb_k;
  bpIndex_b[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_b, frac_b,
    Z1_Sim_ConstP.InterpolationUsingPrelookup5__i, Z1_Sim_ConstP.pooled24);
  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S27>/Gain1' */
    Z1_Sim_B.Gain1 = 0.017453292519943295 * rtb_ail;

    /* Gain: '<S28>/Gain1' */
    Z1_Sim_B.Gain1_n = 0.017453292519943295 * rtb_elev;
  }

  /* Product: '<S26>/Product' */
  rtb_Product_na[0] = rtb_fcn2 * Z1_Sim_B.Gain1;
  rtb_Product_na[1] = rtb_q3dot * Z1_Sim_B.Gain1;
  rtb_Product_na[2] = rtb_q2dot * Z1_Sim_B.Gain1;
  rtb_Product_na[3] = rtb_q1dot * Z1_Sim_B.Gain1;
  rtb_Product_na[4] = rtb_q0dot * Z1_Sim_B.Gain1;
  rtb_Product_na[5] = rtb_HighGainQuaternionNormaliza * Z1_Sim_B.Gain1;

  /* Interpolation_n-D generated from: '<S37>/Interpolation Using Prelookup' */
  frac_c[0] = rtb_InterpolationUsingPrelook_m;
  frac_c[1] = rtb_sigma_ugsigma_vg;
  frac_c[2] = rtb_f_n;
  bpIndex_c[0] = rtb_k_b;
  bpIndex_c[1] = rtb_k;
  bpIndex_c[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_c, frac_c,
    Z1_Sim_ConstP.InterpolationUsingPrelookup_1_m, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S37>/Interpolation Using Prelookup1' */
  frac_d[0] = rtb_InterpolationUsingPrelook_m;
  frac_d[1] = rtb_sigma_ugsigma_vg;
  frac_d[2] = rtb_f_n;
  bpIndex_d[0] = rtb_k_b;
  bpIndex_d[1] = rtb_k;
  bpIndex_d[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_d, frac_d,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1__n,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S37>/Interpolation Using Prelookup2' */
  frac_e[0] = rtb_InterpolationUsingPrelook_m;
  frac_e[1] = rtb_sigma_ugsigma_vg;
  frac_e[2] = rtb_f_n;
  bpIndex_e[0] = rtb_k_b;
  bpIndex_e[1] = rtb_k;
  bpIndex_e[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_e, frac_e,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2__g,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S37>/CL to Cz' */
  rtb_q1dot = -rtb_q1dot;

  /* Interpolation_n-D generated from: '<S37>/Interpolation Using Prelookup3' */
  frac_f[0] = rtb_InterpolationUsingPrelook_m;
  frac_f[1] = rtb_sigma_ugsigma_vg;
  frac_f[2] = rtb_f_n;
  bpIndex_f[0] = rtb_k_b;
  bpIndex_f[1] = rtb_k;
  bpIndex_f[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_f, frac_f,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3__m,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S37>/Interpolation Using Prelookup4' */
  frac_g[0] = rtb_InterpolationUsingPrelook_m;
  frac_g[1] = rtb_sigma_ugsigma_vg;
  frac_g[2] = rtb_f_n;
  bpIndex_g[0] = rtb_k_b;
  bpIndex_g[1] = rtb_k;
  bpIndex_g[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_g, frac_g,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4__b,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S37>/Interpolation Using Prelookup5' */
  frac_h[0] = rtb_InterpolationUsingPrelook_m;
  frac_h[1] = rtb_sigma_ugsigma_vg;
  frac_h[2] = rtb_f_n;
  bpIndex_h[0] = rtb_k_b;
  bpIndex_h[1] = rtb_k;
  bpIndex_h[2] = rtb_k_e;
  rtb_fcn2 = intrp3d_l(bpIndex_h, frac_h,
                       Z1_Sim_ConstP.InterpolationUsingPrelookup5__b,
                       Z1_Sim_ConstP.pooled24);

  /* Product: '<S32>/Product' */
  rtb_ail = rtb_HighGainQuaternionNormaliza * Z1_Sim_B.Gain1_n;
  rtb_elev = rtb_q0dot * Z1_Sim_B.Gain1_n;
  rtb_Product_d_idx_2 = rtb_q1dot * Z1_Sim_B.Gain1_n;
  rtb_Product_d_idx_3 = rtb_q2dot * Z1_Sim_B.Gain1_n;
  rtb_Product_d_idx_4 = rtb_q3dot * Z1_Sim_B.Gain1_n;
  rtb_Product_d_idx_5 = rtb_fcn2 * Z1_Sim_B.Gain1_n;

  /* Interpolation_n-D generated from: '<S40>/Interpolation Using Prelookup' */
  frac_i[0] = rtb_InterpolationUsingPrelook_m;
  frac_i[1] = rtb_sigma_ugsigma_vg;
  frac_i[2] = rtb_f_n;
  bpIndex_i[0] = rtb_k_b;
  bpIndex_i[1] = rtb_k;
  bpIndex_i[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_i, frac_i,
    Z1_Sim_ConstP.InterpolationUsingPrelookup_1_c, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S40>/Interpolation Using Prelookup1' */
  frac_j[0] = rtb_InterpolationUsingPrelook_m;
  frac_j[1] = rtb_sigma_ugsigma_vg;
  frac_j[2] = rtb_f_n;
  bpIndex_j[0] = rtb_k_b;
  bpIndex_j[1] = rtb_k;
  bpIndex_j[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_j, frac_j,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1_no,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S40>/Interpolation Using Prelookup2' */
  frac_k[0] = rtb_InterpolationUsingPrelook_m;
  frac_k[1] = rtb_sigma_ugsigma_vg;
  frac_k[2] = rtb_f_n;
  bpIndex_k[0] = rtb_k_b;
  bpIndex_k[1] = rtb_k;
  bpIndex_k[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_k, frac_k,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2_gd,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S40>/CL to Cz' */
  rtb_q1dot = -rtb_q1dot;

  /* Interpolation_n-D generated from: '<S40>/Interpolation Using Prelookup3' */
  frac_l[0] = rtb_InterpolationUsingPrelook_m;
  frac_l[1] = rtb_sigma_ugsigma_vg;
  frac_l[2] = rtb_f_n;
  bpIndex_l[0] = rtb_k_b;
  bpIndex_l[1] = rtb_k;
  bpIndex_l[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_l, frac_l,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3_kk,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S40>/Interpolation Using Prelookup4' */
  frac_m[0] = rtb_InterpolationUsingPrelook_m;
  frac_m[1] = rtb_sigma_ugsigma_vg;
  frac_m[2] = rtb_f_n;
  bpIndex_m[0] = rtb_k_b;
  bpIndex_m[1] = rtb_k;
  bpIndex_m[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_m, frac_m,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4__c,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S40>/Interpolation Using Prelookup5' */
  frac_n[0] = rtb_InterpolationUsingPrelook_m;
  frac_n[1] = rtb_sigma_ugsigma_vg;
  frac_n[2] = rtb_f_n;
  bpIndex_n[0] = rtb_k_b;
  bpIndex_n[1] = rtb_k;
  bpIndex_n[2] = rtb_k_e;
  rtb_fcn2 = intrp3d_l(bpIndex_n, frac_n,
                       Z1_Sim_ConstP.InterpolationUsingPrelookup5__g,
                       Z1_Sim_ConstP.pooled24);
  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S29>/Gain1' */
    Z1_Sim_B.Gain1_j = 0.017453292519943295 * rtb_rud;

    /* Gain: '<S30>/Gain1' */
    Z1_Sim_B.Gain1_nt = 0.017453292519943295 * rtb_flaps;
  }

  /* Product: '<S35>/Product' */
  rtb_Product_ee_idx_0 = rtb_HighGainQuaternionNormaliza * Z1_Sim_B.Gain1_j;
  rtb_Product_ee_idx_1 = rtb_q0dot * Z1_Sim_B.Gain1_j;
  rtb_Product_ee_idx_2 = rtb_q1dot * Z1_Sim_B.Gain1_j;
  rtb_Product_ee_idx_3 = rtb_q2dot * Z1_Sim_B.Gain1_j;
  rtb_Product_ee_idx_4 = rtb_q3dot * Z1_Sim_B.Gain1_j;
  rtb_rud = rtb_fcn2 * Z1_Sim_B.Gain1_j;

  /* Interpolation_n-D generated from: '<S38>/Interpolation Using Prelookup' */
  frac_o[0] = rtb_InterpolationUsingPrelook_m;
  frac_o[1] = rtb_sigma_ugsigma_vg;
  frac_o[2] = rtb_f_n;
  bpIndex_o[0] = rtb_k_b;
  bpIndex_o[1] = rtb_k;
  bpIndex_o[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_o, frac_o,
    Z1_Sim_ConstP.InterpolationUsingPrelookup_1_p, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S38>/Interpolation Using Prelookup1' */
  frac_p[0] = rtb_InterpolationUsingPrelook_m;
  frac_p[1] = rtb_sigma_ugsigma_vg;
  frac_p[2] = rtb_f_n;
  bpIndex_p[0] = rtb_k_b;
  bpIndex_p[1] = rtb_k;
  bpIndex_p[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_p, frac_p,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1__m,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S38>/Interpolation Using Prelookup2' */
  frac_q[0] = rtb_InterpolationUsingPrelook_m;
  frac_q[1] = rtb_sigma_ugsigma_vg;
  frac_q[2] = rtb_f_n;
  bpIndex_q[0] = rtb_k_b;
  bpIndex_q[1] = rtb_k;
  bpIndex_q[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_q, frac_q,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2__l,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S38>/CL to Cz' */
  rtb_q1dot = -rtb_q1dot;

  /* Interpolation_n-D generated from: '<S38>/Interpolation Using Prelookup3' */
  frac_r[0] = rtb_InterpolationUsingPrelook_m;
  frac_r[1] = rtb_sigma_ugsigma_vg;
  frac_r[2] = rtb_f_n;
  bpIndex_r[0] = rtb_k_b;
  bpIndex_r[1] = rtb_k;
  bpIndex_r[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_r, frac_r,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3__i,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S38>/Interpolation Using Prelookup4' */
  frac_s[0] = rtb_InterpolationUsingPrelook_m;
  frac_s[1] = rtb_sigma_ugsigma_vg;
  frac_s[2] = rtb_f_n;
  bpIndex_s[0] = rtb_k_b;
  bpIndex_s[1] = rtb_k;
  bpIndex_s[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_s, frac_s,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4__a,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S38>/Interpolation Using Prelookup5' */
  frac_t[0] = rtb_InterpolationUsingPrelook_m;
  frac_t[1] = rtb_sigma_ugsigma_vg;
  frac_t[2] = rtb_f_n;
  bpIndex_t[0] = rtb_k_b;
  bpIndex_t[1] = rtb_k;
  bpIndex_t[2] = rtb_k_e;
  rtb_fcn2 = intrp3d_l(bpIndex_t, frac_t,
                       Z1_Sim_ConstP.InterpolationUsingPrelookup5__f,
                       Z1_Sim_ConstP.pooled24);

  /* Product: '<S33>/Product' */
  rtb_Divide_idx_0 = rtb_HighGainQuaternionNormaliza * Z1_Sim_B.Gain1_nt;
  rtb_Divide_idx_1 = rtb_q0dot * Z1_Sim_B.Gain1_nt;
  rtb_Divide_idx_2 = rtb_q1dot * Z1_Sim_B.Gain1_nt;
  rtb_Divide_idx_3 = rtb_q2dot * Z1_Sim_B.Gain1_nt;
  rtb_Divide_idx_4 = rtb_q3dot * Z1_Sim_B.Gain1_nt;
  rtb_flaps = rtb_fcn2 * Z1_Sim_B.Gain1_nt;

  /* Interpolation_n-D generated from: '<S39>/Interpolation Using Prelookup' */
  frac_u[0] = rtb_InterpolationUsingPrelook_m;
  frac_u[1] = rtb_sigma_ugsigma_vg;
  frac_u[2] = rtb_f_n;
  bpIndex_u[0] = rtb_k_b;
  bpIndex_u[1] = rtb_k;
  bpIndex_u[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_u, frac_u,
    Z1_Sim_ConstP.InterpolationUsingPrelookup__az, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S39>/Interpolation Using Prelookup1' */
  frac_v[0] = rtb_InterpolationUsingPrelook_m;
  frac_v[1] = rtb_sigma_ugsigma_vg;
  frac_v[2] = rtb_f_n;
  bpIndex_v[0] = rtb_k_b;
  bpIndex_v[1] = rtb_k;
  bpIndex_v[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_v, frac_v,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1__k,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S39>/Interpolation Using Prelookup2' */
  frac_w[0] = rtb_InterpolationUsingPrelook_m;
  frac_w[1] = rtb_sigma_ugsigma_vg;
  frac_w[2] = rtb_f_n;
  bpIndex_w[0] = rtb_k_b;
  bpIndex_w[1] = rtb_k;
  bpIndex_w[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_w, frac_w,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2__k,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S39>/CL to Cz' */
  rtb_q1dot = -rtb_q1dot;

  /* Interpolation_n-D generated from: '<S39>/Interpolation Using Prelookup3' */
  frac_x[0] = rtb_InterpolationUsingPrelook_m;
  frac_x[1] = rtb_sigma_ugsigma_vg;
  frac_x[2] = rtb_f_n;
  bpIndex_x[0] = rtb_k_b;
  bpIndex_x[1] = rtb_k;
  bpIndex_x[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_x, frac_x,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3__a,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S39>/Interpolation Using Prelookup4' */
  frac_y[0] = rtb_InterpolationUsingPrelook_m;
  frac_y[1] = rtb_sigma_ugsigma_vg;
  frac_y[2] = rtb_f_n;
  bpIndex_y[0] = rtb_k_b;
  bpIndex_y[1] = rtb_k;
  bpIndex_y[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_y, frac_y,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4__o,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S39>/Interpolation Using Prelookup5' */
  frac_z[0] = rtb_InterpolationUsingPrelook_m;
  frac_z[1] = rtb_sigma_ugsigma_vg;
  frac_z[2] = rtb_f_n;
  bpIndex_z[0] = rtb_k_b;
  bpIndex_z[1] = rtb_k;
  bpIndex_z[2] = rtb_k_e;
  rtb_fcn2 = intrp3d_l(bpIndex_z, frac_z,
                       Z1_Sim_ConstP.InterpolationUsingPrelookup5__l,
                       Z1_Sim_ConstP.pooled24);
  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
    /* Gain: '<S31>/Gain1' */
    Z1_Sim_B.Gain1_h = 0.017453292519943295 * rtb_ailOut;
  }

  /* Sum: '<S24>/Sum' incorporates:
   *  Product: '<S34>/Product'
   */
  rtb_ailOut = (((rtb_Product_na[0] + rtb_ail) + rtb_Product_ee_idx_0) +
                rtb_Divide_idx_0) + rtb_HighGainQuaternionNormaliza *
    Z1_Sim_B.Gain1_h;
  rtb_Product_ee_idx_1 = (((rtb_Product_na[1] + rtb_elev) + rtb_Product_ee_idx_1)
    + rtb_Divide_idx_1) + rtb_q0dot * Z1_Sim_B.Gain1_h;
  rtb_Product_ee_idx_2 = (((rtb_Product_na[2] + rtb_Product_d_idx_2) +
    rtb_Product_ee_idx_2) + rtb_Divide_idx_2) + rtb_q1dot * Z1_Sim_B.Gain1_h;
  rtb_Product_ee_idx_3 = (((rtb_Product_na[3] + rtb_Product_d_idx_3) +
    rtb_Product_ee_idx_3) + rtb_Divide_idx_3) + rtb_q2dot * Z1_Sim_B.Gain1_h;
  rtb_Product_ee_idx_4 = (((rtb_Product_na[4] + rtb_Product_d_idx_4) +
    rtb_Product_ee_idx_4) + rtb_Divide_idx_4) + rtb_q3dot * Z1_Sim_B.Gain1_h;
  rtb_flaps = (((rtb_Product_na[5] + rtb_Product_d_idx_5) + rtb_rud) + rtb_flaps)
    + rtb_fcn2 * Z1_Sim_B.Gain1_h;

  /* Interpolation_n-D generated from: '<S43>/Interpolation Using Prelookup' */
  frac_10[0] = rtb_InterpolationUsingPrelook_m;
  frac_10[1] = rtb_sigma_ugsigma_vg;
  frac_10[2] = rtb_f_n;
  bpIndex_10[0] = rtb_k_b;
  bpIndex_10[1] = rtb_k;
  bpIndex_10[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_10, frac_10,
    Z1_Sim_ConstP.InterpolationUsingPrelookup_1_e, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S43>/Interpolation Using Prelookup1' */
  frac_11[0] = rtb_InterpolationUsingPrelook_m;
  frac_11[1] = rtb_sigma_ugsigma_vg;
  frac_11[2] = rtb_f_n;
  bpIndex_11[0] = rtb_k_b;
  bpIndex_11[1] = rtb_k;
  bpIndex_11[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_11, frac_11,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1__h,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S43>/Interpolation Using Prelookup2' */
  frac_12[0] = rtb_InterpolationUsingPrelook_m;
  frac_12[1] = rtb_sigma_ugsigma_vg;
  frac_12[2] = rtb_f_n;
  bpIndex_12[0] = rtb_k_b;
  bpIndex_12[1] = rtb_k;
  bpIndex_12[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_12, frac_12,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2_kh,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S43>/CL to Cz' */
  rtb_q1dot = -rtb_q1dot;

  /* Interpolation_n-D generated from: '<S43>/Interpolation Using Prelookup3' */
  frac_13[0] = rtb_InterpolationUsingPrelook_m;
  frac_13[1] = rtb_sigma_ugsigma_vg;
  frac_13[2] = rtb_f_n;
  bpIndex_13[0] = rtb_k_b;
  bpIndex_13[1] = rtb_k;
  bpIndex_13[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_13, frac_13,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3__f,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S43>/Interpolation Using Prelookup4' */
  frac_14[0] = rtb_InterpolationUsingPrelook_m;
  frac_14[1] = rtb_sigma_ugsigma_vg;
  frac_14[2] = rtb_f_n;
  bpIndex_14[0] = rtb_k_b;
  bpIndex_14[1] = rtb_k;
  bpIndex_14[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_14, frac_14,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4_jn,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S43>/Interpolation Using Prelookup5' */
  frac_15[0] = rtb_InterpolationUsingPrelook_m;
  frac_15[1] = rtb_sigma_ugsigma_vg;
  frac_15[2] = rtb_f_n;
  bpIndex_15[0] = rtb_k_b;
  bpIndex_15[1] = rtb_k;
  bpIndex_15[2] = rtb_k_e;
  rtb_fcn2 = intrp3d_l(bpIndex_15, frac_15,
                       Z1_Sim_ConstP.InterpolationUsingPrelookup5__d,
                       Z1_Sim_ConstP.pooled24);

  /* Product: '<S41>/Product' */
  rtb_Product_na[0] = rtb_HighGainQuaternionNormaliza *
    Z1_Sim_B.DataStoreRead.thr;
  rtb_Product_na[1] = rtb_q0dot * Z1_Sim_B.DataStoreRead.thr;
  rtb_Product_na[2] = rtb_q1dot * Z1_Sim_B.DataStoreRead.thr;
  rtb_Product_na[3] = rtb_q2dot * Z1_Sim_B.DataStoreRead.thr;
  rtb_Product_na[4] = rtb_q3dot * Z1_Sim_B.DataStoreRead.thr;
  rtb_Product_na[5] = rtb_fcn2 * Z1_Sim_B.DataStoreRead.thr;

  /* Interpolation_n-D generated from: '<S44>/Interpolation Using Prelookup' */
  frac_16[0] = rtb_InterpolationUsingPrelook_m;
  frac_16[1] = rtb_sigma_ugsigma_vg;
  frac_16[2] = rtb_f_n;
  bpIndex_16[0] = rtb_k_b;
  bpIndex_16[1] = rtb_k;
  bpIndex_16[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_16, frac_16,
    Z1_Sim_ConstP.InterpolationUsingPrelookup__aw, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S44>/Interpolation Using Prelookup1' */
  frac_17[0] = rtb_InterpolationUsingPrelook_m;
  frac_17[1] = rtb_sigma_ugsigma_vg;
  frac_17[2] = rtb_f_n;
  bpIndex_17[0] = rtb_k_b;
  bpIndex_17[1] = rtb_k;
  bpIndex_17[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_17, frac_17,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1__p,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S44>/Interpolation Using Prelookup2' */
  frac_18[0] = rtb_InterpolationUsingPrelook_m;
  frac_18[1] = rtb_sigma_ugsigma_vg;
  frac_18[2] = rtb_f_n;
  bpIndex_18[0] = rtb_k_b;
  bpIndex_18[1] = rtb_k;
  bpIndex_18[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_18, frac_18,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2__o,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S44>/CL to Cz' */
  rtb_q1dot = -rtb_q1dot;

  /* Interpolation_n-D generated from: '<S44>/Interpolation Using Prelookup3' */
  frac_19[0] = rtb_InterpolationUsingPrelook_m;
  frac_19[1] = rtb_sigma_ugsigma_vg;
  frac_19[2] = rtb_f_n;
  bpIndex_19[0] = rtb_k_b;
  bpIndex_19[1] = rtb_k;
  bpIndex_19[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_19, frac_19,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3__o,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S44>/Interpolation Using Prelookup4' */
  frac_1a[0] = rtb_InterpolationUsingPrelook_m;
  frac_1a[1] = rtb_sigma_ugsigma_vg;
  frac_1a[2] = rtb_f_n;
  bpIndex_1a[0] = rtb_k_b;
  bpIndex_1a[1] = rtb_k;
  bpIndex_1a[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_1a, frac_1a,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4__k,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S44>/Interpolation Using Prelookup5' */
  frac_1b[0] = rtb_InterpolationUsingPrelook_m;
  frac_1b[1] = rtb_sigma_ugsigma_vg;
  frac_1b[2] = rtb_f_n;
  bpIndex_1b[0] = rtb_k_b;
  bpIndex_1b[1] = rtb_k;
  bpIndex_1b[2] = rtb_k_e;
  rtb_fcn2 = intrp3d_l(bpIndex_1b, frac_1b,
                       Z1_Sim_ConstP.InterpolationUsingPrelookup5_fc,
                       Z1_Sim_ConstP.pooled24);

  /* Product: '<S42>/Product' incorporates:
   *  Sum: '<S19>/Add'
   *  Sum: '<S25>/Sum'
   */
  rtb_Sum_a_0[0] = (rtb_HighGainQuaternionNormaliza *
                    Z1_Sim_B.DataStoreRead.thrDiff + rtb_Product_na[0]) +
    rtb_ailOut;
  rtb_Sum_a_0[1] = (rtb_q0dot * Z1_Sim_B.DataStoreRead.thrDiff + rtb_Product_na
                    [1]) + rtb_Product_ee_idx_1;
  rtb_Sum_a_0[2] = (rtb_q1dot * Z1_Sim_B.DataStoreRead.thrDiff + rtb_Product_na
                    [2]) + rtb_Product_ee_idx_2;
  rtb_Sum_a_0[3] = (rtb_q2dot * Z1_Sim_B.DataStoreRead.thrDiff + rtb_Product_na
                    [3]) + rtb_Product_ee_idx_3;
  rtb_Sum_a_0[4] = (rtb_q3dot * Z1_Sim_B.DataStoreRead.thrDiff + rtb_Product_na
                    [4]) + rtb_Product_ee_idx_4;
  rtb_Sum_a_0[5] = (rtb_fcn2 * Z1_Sim_B.DataStoreRead.thrDiff + rtb_Product_na[5])
    + rtb_flaps;

  /* Sum: '<S19>/Add' */
  for (i = 0; i < 6; i++) {
    rtb_Product_na[i] = rtb_Sum_a_0[i];
  }

  /* Interpolation_n-D generated from: '<S49>/Interpolation Using Prelookup' */
  frac_1c[0] = rtb_InterpolationUsingPrelook_m;
  frac_1c[1] = rtb_sigma_ugsigma_vg;
  frac_1c[2] = rtb_f_n;
  bpIndex_1c[0] = rtb_k_b;
  bpIndex_1c[1] = rtb_k;
  bpIndex_1c[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_1c, frac_1c,
    Z1_Sim_ConstP.InterpolationUsingPrelookup_1_l, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S49>/Interpolation Using Prelookup1' */
  frac_1d[0] = rtb_InterpolationUsingPrelook_m;
  frac_1d[1] = rtb_sigma_ugsigma_vg;
  frac_1d[2] = rtb_f_n;
  bpIndex_1d[0] = rtb_k_b;
  bpIndex_1d[1] = rtb_k;
  bpIndex_1d[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_1d, frac_1d,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1__i,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S49>/Interpolation Using Prelookup2' */
  frac_1e[0] = rtb_InterpolationUsingPrelook_m;
  frac_1e[1] = rtb_sigma_ugsigma_vg;
  frac_1e[2] = rtb_f_n;
  bpIndex_1e[0] = rtb_k_b;
  bpIndex_1e[1] = rtb_k;
  bpIndex_1e[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_1e, frac_1e,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2_kt,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S49>/CL to Cz' */
  rtb_q1dot = -rtb_q1dot;

  /* Interpolation_n-D generated from: '<S49>/Interpolation Using Prelookup3' */
  frac_1f[0] = rtb_InterpolationUsingPrelook_m;
  frac_1f[1] = rtb_sigma_ugsigma_vg;
  frac_1f[2] = rtb_f_n;
  bpIndex_1f[0] = rtb_k_b;
  bpIndex_1f[1] = rtb_k;
  bpIndex_1f[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_1f, frac_1f,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3__g,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S49>/Interpolation Using Prelookup4' */
  frac_1g[0] = rtb_InterpolationUsingPrelook_m;
  frac_1g[1] = rtb_sigma_ugsigma_vg;
  frac_1g[2] = rtb_f_n;
  bpIndex_1g[0] = rtb_k_b;
  bpIndex_1g[1] = rtb_k;
  bpIndex_1g[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_1g, frac_1g,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4__f,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S49>/Interpolation Using Prelookup5' */
  frac_1h[0] = rtb_InterpolationUsingPrelook_m;
  frac_1h[1] = rtb_sigma_ugsigma_vg;
  frac_1h[2] = rtb_f_n;
  bpIndex_1h[0] = rtb_k_b;
  bpIndex_1h[1] = rtb_k;
  bpIndex_1h[2] = rtb_k_e;
  rtb_fcn2 = intrp3d_l(bpIndex_1h, frac_1h,
                       Z1_Sim_ConstP.InterpolationUsingPrelookup5__k,
                       Z1_Sim_ConstP.pooled24);

  /* Outputs for Enabled SubSystem: '<S171>/Hqgw' incorporates:
   *  EnablePort: '<S183>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S171>/Hpgw' incorporates:
   *  EnablePort: '<S182>/Enable'
   */
  if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
      (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
    if (rtmIsMajorTimeStep((&Z1_Sim_M)) && Z1_Sim_DW.Hpgw_MODE) {
      /* Disable for Outport: '<S182>/pgw' */
      Z1_Sim_B.sigma_w[0] = 0.0;
      Z1_Sim_B.sigma_w[1] = 0.0;
      Z1_Sim_DW.Hpgw_MODE = false;
    }

    if (rtmIsMajorTimeStep((&Z1_Sim_M)) && Z1_Sim_DW.Hqgw_MODE) {
      /* Disable for Outport: '<S183>/qgw' */
      Z1_Sim_B.w_b[0] = 0.0;
      Z1_Sim_B.w_b[1] = 0.0;
      Z1_Sim_DW.Hqgw_MODE = false;
    }
  }

  /* End of Outputs for SubSystem: '<S171>/Hqgw' */
  if (Z1_Sim_DW.Hpgw_MODE) {
    /* Fcn: '<S182>/sqrt(0.8//V)' */
    rtb_Product_d_idx_2 = 0.8 / rtb_UnitConversion_i;
    if (rtb_Product_d_idx_2 < 0.0) {
      rtb_Product_d_idx_2 = -std::sqrt(-rtb_Product_d_idx_2);
    } else {
      rtb_Product_d_idx_2 = std::sqrt(rtb_Product_d_idx_2);
    }

    /* Product: '<S182>/sigma_w' incorporates:
     *  Integrator: '<S182>/pgw_p'
     */
    Z1_Sim_B.sigma_w[0] = 1.6404199475065615 * Z1_Sim_X.pgw_p_CSTATE[0];
    Z1_Sim_B.sigma_w[1] = rtb_MediumHighAltitudeIntensity *
      Z1_Sim_X.pgw_p_CSTATE[1];

    /* Product: '<S182>/w3' */
    rtb_ail = rtb_UnitConversion_i * 0.059847340050885565;

    /* Product: '<S182>/w' incorporates:
     *  Fcn: '<S182>/sqrt(0.8//V)'
     *  Integrator: '<S182>/pgw_p'
     *  Math: '<S182>/L^1//3'
     *  Product: '<S182>/Lug//V1'
     *  Product: '<S182>/w1'
     *  Product: '<S182>/w2'
     *  Sum: '<S182>/Sum'
     */
    Z1_Sim_B.w_hh[0] = (rtb_Product_d_idx_2 / rt_powd_snf(frac[0],
      0.33333333333333331) * 0.62542342293925646 * Z1_Sim_B.Product_j[3] -
                        Z1_Sim_X.pgw_p_CSTATE[0]) * rtb_ail;

    /* Math: '<S182>/L^1//3' incorporates:
     *  Gain: '<S178>/Lw'
     */
    if (Z1_Sim_ConstB.UnitConversion_cm < 0.0) {
      rtb_ailOut = -rt_powd_snf(-Z1_Sim_ConstB.UnitConversion_cm,
        0.33333333333333331);
    } else {
      rtb_ailOut = rt_powd_snf(Z1_Sim_ConstB.UnitConversion_cm,
        0.33333333333333331);
    }

    /* Product: '<S182>/w' incorporates:
     *  Fcn: '<S182>/sqrt(0.8//V)'
     *  Integrator: '<S182>/pgw_p'
     *  Product: '<S182>/Lug//V1'
     *  Product: '<S182>/w1'
     *  Product: '<S182>/w2'
     *  Sum: '<S182>/Sum'
     */
    Z1_Sim_B.w_hh[1] = (rtb_Product_d_idx_2 / rtb_ailOut * 0.62542342293925646 *
                        Z1_Sim_B.Product_j[3] - Z1_Sim_X.pgw_p_CSTATE[1]) *
      rtb_ail;
  }

  /* End of Outputs for SubSystem: '<S171>/Hpgw' */

  /* Outputs for Enabled SubSystem: '<S171>/Hqgw' incorporates:
   *  EnablePort: '<S183>/Enable'
   */
  if (Z1_Sim_DW.Hqgw_MODE) {
    /* Gain: '<S183>/pi//4' */
    rtb_ail = 0.78539816339744828 * rtb_UnitConversion_i;

    /* Product: '<S183>/w' incorporates:
     *  Integrator: '<S183>/qgw_p'
     *  Product: '<S183>/wg//V'
     *  Sum: '<S183>/Sum'
     */
    Z1_Sim_B.w_b[0] = (Z1_Sim_B.LwgV1[0] / rtb_UnitConversion_i -
                       Z1_Sim_X.qgw_p_CSTATE[0]) * (rtb_ail / 13.123359580052492);
    Z1_Sim_B.w_b[1] = (Z1_Sim_B.LwgV1[1] / rtb_UnitConversion_i -
                       Z1_Sim_X.qgw_p_CSTATE[1]) * (rtb_ail / 13.123359580052492);
  }

  /* End of Outputs for SubSystem: '<S171>/Hqgw' */

  /* Outputs for Enabled SubSystem: '<S171>/Hrgw' incorporates:
   *  EnablePort: '<S184>/Enable'
   */
  if ((rtmIsMajorTimeStep((&Z1_Sim_M)) &&
       (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) && rtmIsMajorTimeStep
      ((&Z1_Sim_M)) && Z1_Sim_DW.Hrgw_MODE) {
    /* Disable for Outport: '<S184>/rgw' */
    Z1_Sim_B.UnaryMinus[0] = 0.0;
    Z1_Sim_B.UnaryMinus[1] = 0.0;
    Z1_Sim_DW.Hrgw_MODE = false;
  }

  if (Z1_Sim_DW.Hrgw_MODE) {
    /* Gain: '<S184>/pi//3' */
    rtb_ail = 1.0471975511965976 * rtb_UnitConversion_i;

    /* Product: '<S184>/w' incorporates:
     *  Integrator: '<S184>/rgw_p'
     *  Product: '<S184>/vg//V'
     *  Sum: '<S184>/Sum'
     */
    Z1_Sim_B.w_jk[0] = (Z1_Sim_B.w1[0] / rtb_UnitConversion_i -
                        Z1_Sim_X.rgw_p_CSTATE[0]) * (rtb_ail /
      13.123359580052492);

    /* UnaryMinus: '<S184>/Unary Minus' */
    Z1_Sim_B.UnaryMinus[0] = -Z1_Sim_B.w_jk[0];

    /* Product: '<S184>/w' incorporates:
     *  Integrator: '<S184>/rgw_p'
     *  Product: '<S184>/vg//V'
     *  Sum: '<S184>/Sum'
     */
    Z1_Sim_B.w_jk[1] = (Z1_Sim_B.w1[1] / rtb_UnitConversion_i -
                        Z1_Sim_X.rgw_p_CSTATE[1]) * (rtb_ail /
      13.123359580052492);

    /* UnaryMinus: '<S184>/Unary Minus' */
    Z1_Sim_B.UnaryMinus[1] = -Z1_Sim_B.w_jk[1];
  }

  /* End of Outputs for SubSystem: '<S171>/Hrgw' */

  /* If: '<S176>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    if (rtb_Product_c <= 1000.0) {
      rtAction = 0;
    } else if (rtb_Product_c >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    Z1_Sim_DW.ifHeightMaxlowaltitudeelseifH_e = rtAction;
  } else {
    rtAction = Z1_Sim_DW.ifHeightMaxlowaltitudeelseifH_e;
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S176>/Low altitude  rates' incorporates:
     *  ActionPort: '<S191>/Action Port'
     */
    /* Sum: '<S197>/Sum' incorporates:
     *  Product: '<S197>/Product1'
     *  Product: '<S197>/Product2'
     */
    frac_0[0] = Z1_Sim_B.sigma_w[0] - 0.0 * Z1_Sim_B.w_b[0];

    /* Sum: '<S197>/Sum1' incorporates:
     *  Product: '<S197>/Product1'
     *  Product: '<S197>/Product2'
     */
    frac_0[1] = 0.0 * Z1_Sim_B.sigma_w[0] + Z1_Sim_B.w_b[0];

    /* Reshape: '<S196>/Reshape1' incorporates:
     *  Product: '<S196>/Product'
     *  SignalConversion generated from: '<S196>/Vector Concatenate'
     */
    for (i = 0; i < 3; i++) {
      rtb_Sum_gb[i] = Z1_Sim_B.VectorConcatenate[i + 6] * Z1_Sim_B.UnaryMinus[0]
        + (Z1_Sim_B.VectorConcatenate[i + 3] * frac_0[1] +
           Z1_Sim_B.VectorConcatenate[i] * frac_0[0]);
    }

    /* End of Reshape: '<S196>/Reshape1' */
    /* End of Outputs for SubSystem: '<S176>/Low altitude  rates' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S176>/Medium//High  altitude rates' incorporates:
     *  ActionPort: '<S192>/Action Port'
     */
    /* Gain: '<S192>/Gain' */
    rtb_Sum_gb[0] = Z1_Sim_B.sigma_w[1];
    rtb_Sum_gb[1] = Z1_Sim_B.w_b[1];
    rtb_Sum_gb[2] = Z1_Sim_B.UnaryMinus[1];

    /* End of Outputs for SubSystem: '<S176>/Medium//High  altitude rates' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S176>/Interpolate  rates' incorporates:
     *  ActionPort: '<S190>/Action Port'
     */
    /* Sum: '<S195>/Sum' incorporates:
     *  Product: '<S195>/Product1'
     *  Product: '<S195>/Product2'
     */
    rtb_Sum_gb[0] = Z1_Sim_B.sigma_w[0] - 0.0 * Z1_Sim_B.w_b[0];

    /* Sum: '<S195>/Sum1' incorporates:
     *  Product: '<S195>/Product1'
     *  Product: '<S195>/Product2'
     */
    rtb_Sum_gb[1] = 0.0 * Z1_Sim_B.sigma_w[0] + Z1_Sim_B.w_b[0];

    /* Product: '<S194>/Product' incorporates:
     *  SignalConversion generated from: '<S194>/Vector Concatenate'
     */
    for (i = 0; i < 3; i++) {
      frac_0[i] = Z1_Sim_B.VectorConcatenate[i + 6] * Z1_Sim_B.UnaryMinus[0] +
        (Z1_Sim_B.VectorConcatenate[i + 3] * rtb_Sum_gb[1] +
         Z1_Sim_B.VectorConcatenate[i] * rtb_Sum_gb[0]);
    }

    /* End of Product: '<S194>/Product' */

    /* Sum: '<S190>/Sum1' incorporates:
     *  Constant: '<S190>/max_height_low'
     */
    rtb_Product_c -= 1000.0;

    /* Sum: '<S190>/Sum3' incorporates:
     *  Product: '<S190>/Product1'
     *  Sum: '<S190>/Sum2'
     */
    rtb_Sum_gb[0] = (Z1_Sim_B.sigma_w[1] - frac_0[0]) * rtb_Product_c / 1000.0 +
      frac_0[0];
    rtb_Sum_gb[1] = (Z1_Sim_B.w_b[1] - frac_0[1]) * rtb_Product_c / 1000.0 +
      frac_0[1];
    rtb_Sum_gb[2] = (Z1_Sim_B.UnaryMinus[1] - frac_0[2]) * rtb_Product_c /
      1000.0 + frac_0[2];

    /* End of Outputs for SubSystem: '<S176>/Interpolate  rates' */
    break;
  }

  /* End of If: '<S176>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */

  /* Sum: '<S9>/Sum2' incorporates:
   *  Integrator: '<S7>/p,q,r '
   */
  rtb_Sum_gb[0] += Z1_Sim_X.p[0];
  rtb_Sum_gb[1] += Z1_Sim_X.p[1];
  rtb_Product_d_idx_2 = Z1_Sim_X.p[2] + rtb_Sum_gb[2];

  /* Product: '<S21>/Body to Stab' */
  for (i = 0; i < 3; i++) {
    rtb_Sum2_m[i] = rtb_VectorConcatenate_m[i + 6] * rtb_Product_d_idx_2 +
      (rtb_VectorConcatenate_m[i + 3] * rtb_Sum_gb[1] +
       rtb_VectorConcatenate_m[i] * rtb_Sum_gb[0]);
  }

  /* End of Product: '<S21>/Body to Stab' */

  /* Gain: '<S46>/Reference Span' incorporates:
   *  Product: '<S46>/Product'
   */
  rtb_ailOut = rtb_HighGainQuaternionNormaliza * rtb_Sum2_m[0] * 5.0;
  rtb_Product_ee_idx_1 = rtb_q0dot * rtb_Sum2_m[0] * 5.0;
  rtb_Product_ee_idx_2 = rtb_q1dot * rtb_Sum2_m[0] * 5.0;
  rtb_Product_ee_idx_3 = rtb_q2dot * rtb_Sum2_m[0] * 5.0;
  rtb_Product_ee_idx_4 = rtb_q3dot * rtb_Sum2_m[0] * 5.0;
  rtb_flaps = rtb_fcn2 * rtb_Sum2_m[0] * 5.0;

  /* Interpolation_n-D generated from: '<S50>/Interpolation Using Prelookup' */
  frac_1i[0] = rtb_InterpolationUsingPrelook_m;
  frac_1i[1] = rtb_sigma_ugsigma_vg;
  frac_1i[2] = rtb_f_n;
  bpIndex_1i[0] = rtb_k_b;
  bpIndex_1i[1] = rtb_k;
  bpIndex_1i[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_1i, frac_1i,
    Z1_Sim_ConstP.InterpolationUsingPrelookup__lv, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S50>/Interpolation Using Prelookup1' */
  frac_1j[0] = rtb_InterpolationUsingPrelook_m;
  frac_1j[1] = rtb_sigma_ugsigma_vg;
  frac_1j[2] = rtb_f_n;
  bpIndex_1j[0] = rtb_k_b;
  bpIndex_1j[1] = rtb_k;
  bpIndex_1j[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_1j, frac_1j,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1_mi,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S50>/Interpolation Using Prelookup2' */
  frac_1k[0] = rtb_InterpolationUsingPrelook_m;
  frac_1k[1] = rtb_sigma_ugsigma_vg;
  frac_1k[2] = rtb_f_n;
  bpIndex_1k[0] = rtb_k_b;
  bpIndex_1k[1] = rtb_k;
  bpIndex_1k[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_1k, frac_1k,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2__j,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S50>/CL to Cz' */
  rtb_q1dot = -rtb_q1dot;

  /* Interpolation_n-D generated from: '<S50>/Interpolation Using Prelookup3' */
  frac_1l[0] = rtb_InterpolationUsingPrelook_m;
  frac_1l[1] = rtb_sigma_ugsigma_vg;
  frac_1l[2] = rtb_f_n;
  bpIndex_1l[0] = rtb_k_b;
  bpIndex_1l[1] = rtb_k;
  bpIndex_1l[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_1l, frac_1l,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3__e,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S50>/Interpolation Using Prelookup4' */
  frac_1m[0] = rtb_InterpolationUsingPrelook_m;
  frac_1m[1] = rtb_sigma_ugsigma_vg;
  frac_1m[2] = rtb_f_n;
  bpIndex_1m[0] = rtb_k_b;
  bpIndex_1m[1] = rtb_k;
  bpIndex_1m[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_1m, frac_1m,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4__d,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S50>/Interpolation Using Prelookup5' */
  frac_1n[0] = rtb_InterpolationUsingPrelook_m;
  frac_1n[1] = rtb_sigma_ugsigma_vg;
  frac_1n[2] = rtb_f_n;
  bpIndex_1n[0] = rtb_k_b;
  bpIndex_1n[1] = rtb_k;
  bpIndex_1n[2] = rtb_k_e;
  rtb_fcn2 = intrp3d_l(bpIndex_1n, frac_1n,
                       Z1_Sim_ConstP.InterpolationUsingPrelookup5_d3,
                       Z1_Sim_ConstP.pooled24);

  /* Gain: '<S47>/Reference Length' incorporates:
   *  Product: '<S47>/Product'
   */
  rtb_ail = rtb_HighGainQuaternionNormaliza * rtb_Sum2_m[1] * 0.2;
  rtb_elev = rtb_q0dot * rtb_Sum2_m[1] * 0.2;
  rtb_Product_d_idx_2 = rtb_q1dot * rtb_Sum2_m[1] * 0.2;
  rtb_Product_d_idx_3 = rtb_q2dot * rtb_Sum2_m[1] * 0.2;
  rtb_Product_d_idx_4 = rtb_q3dot * rtb_Sum2_m[1] * 0.2;

  /* Interpolation_n-D generated from: '<S51>/Interpolation Using Prelookup' */
  frac_1o[0] = rtb_InterpolationUsingPrelook_m;
  frac_1o[1] = rtb_sigma_ugsigma_vg;
  frac_1o[2] = rtb_f_n;
  bpIndex_1o[0] = rtb_k_b;
  bpIndex_1o[1] = rtb_k;
  bpIndex_1o[2] = rtb_k_e;
  rtb_HighGainQuaternionNormaliza = intrp3d_l(bpIndex_1o, frac_1o,
    Z1_Sim_ConstP.InterpolationUsingPrelookup__lj, Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S51>/Interpolation Using Prelookup1' */
  frac_1p[0] = rtb_InterpolationUsingPrelook_m;
  frac_1p[1] = rtb_sigma_ugsigma_vg;
  frac_1p[2] = rtb_f_n;
  bpIndex_1p[0] = rtb_k_b;
  bpIndex_1p[1] = rtb_k;
  bpIndex_1p[2] = rtb_k_e;
  rtb_q0dot = intrp3d_l(bpIndex_1p, frac_1p,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup1__a,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S51>/Interpolation Using Prelookup2' */
  frac_1q[0] = rtb_InterpolationUsingPrelook_m;
  frac_1q[1] = rtb_sigma_ugsigma_vg;
  frac_1q[2] = rtb_f_n;
  bpIndex_1q[0] = rtb_k_b;
  bpIndex_1q[1] = rtb_k;
  bpIndex_1q[2] = rtb_k_e;
  rtb_q1dot = intrp3d_l(bpIndex_1q, frac_1q,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup2_gy,
                        Z1_Sim_ConstP.pooled24);

  /* Gain: '<S51>/CL to Cz' */
  rtb_q1dot = -rtb_q1dot;

  /* Interpolation_n-D generated from: '<S51>/Interpolation Using Prelookup3' */
  frac_1r[0] = rtb_InterpolationUsingPrelook_m;
  frac_1r[1] = rtb_sigma_ugsigma_vg;
  frac_1r[2] = rtb_f_n;
  bpIndex_1r[0] = rtb_k_b;
  bpIndex_1r[1] = rtb_k;
  bpIndex_1r[2] = rtb_k_e;
  rtb_q2dot = intrp3d_l(bpIndex_1r, frac_1r,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup3_gp,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S51>/Interpolation Using Prelookup4' */
  frac_1s[0] = rtb_InterpolationUsingPrelook_m;
  frac_1s[1] = rtb_sigma_ugsigma_vg;
  frac_1s[2] = rtb_f_n;
  bpIndex_1s[0] = rtb_k_b;
  bpIndex_1s[1] = rtb_k;
  bpIndex_1s[2] = rtb_k_e;
  rtb_q3dot = intrp3d_l(bpIndex_1s, frac_1s,
                        Z1_Sim_ConstP.InterpolationUsingPrelookup4_ad,
                        Z1_Sim_ConstP.pooled24);

  /* Interpolation_n-D generated from: '<S51>/Interpolation Using Prelookup5' */
  frac_1t[0] = rtb_InterpolationUsingPrelook_m;
  frac_1t[1] = rtb_sigma_ugsigma_vg;
  frac_1t[2] = rtb_f_n;
  bpIndex_1t[0] = rtb_k_b;
  bpIndex_1t[1] = rtb_k;
  bpIndex_1t[2] = rtb_k_e;
  rtb_InterpolationUsingPrelook_m = intrp3d_l(bpIndex_1t, frac_1t,
    Z1_Sim_ConstP.InterpolationUsingPrelookup5_k3, Z1_Sim_ConstP.pooled24);

  /* Saturate: '<S21>/Saturation' */
  if (rtb_Saturation <= 0.1) {
    rtb_Saturation = 0.1;
  }

  /* End of Saturate: '<S21>/Saturation' */

  /* Sum: '<S10>/Sum' incorporates:
   *  Gain: '<S47>/Reference Length'
   *  Gain: '<S48>/Reference Span'
   *  Product: '<S21>/Divide'
   *  Product: '<S47>/Product'
   *  Product: '<S48>/Product'
   *  Sum: '<S21>/Sum1'
   */
  rtb_Sum_a_0[0] = (rtb_HighGainQuaternionNormaliza * rtb_Sum2_m[2] * 5.0 +
                    (rtb_ailOut + rtb_ail)) / rtb_Saturation +
    (rtb_InterpolationUsingPrelookup + rtb_Product_na[0]);
  rtb_Sum_a_0[1] = (rtb_q0dot * rtb_Sum2_m[2] * 5.0 + (rtb_Product_ee_idx_1 +
    rtb_elev)) / rtb_Saturation + (rtb_InterpolationUsingPreloo_nk +
    rtb_Product_na[1]);
  rtb_Sum_a_0[2] = (rtb_q1dot * rtb_Sum2_m[2] * 5.0 + (rtb_Product_ee_idx_2 +
    rtb_Product_d_idx_2)) / rtb_Saturation + (rtb_CLtoCz + rtb_Product_na[2]);
  rtb_Sum_a_0[3] = (rtb_q2dot * rtb_Sum2_m[2] * 5.0 + (rtb_Product_ee_idx_3 +
    rtb_Product_d_idx_3)) / rtb_Saturation + (rtb_InterpolationUsingPrelook_a +
    rtb_Product_na[3]);
  rtb_Sum_a_0[4] = (rtb_q3dot * rtb_Sum2_m[2] * 5.0 + (rtb_Product_ee_idx_4 +
    rtb_Product_d_idx_4)) / rtb_Saturation + (rtb_InterpolationUsingPrelook_p +
    rtb_Product_na[4]);
  rtb_Sum_a_0[5] = ((rtb_fcn2 * rtb_Sum2_m[1] * 0.2 + rtb_flaps) +
                    rtb_InterpolationUsingPrelook_m * rtb_Sum2_m[2] * 5.0) /
    rtb_Saturation + (rtb_InterpolationUsingPrelook_h + rtb_Product_na[5]);
  for (i = 0; i < 6; i++) {
    rtb_Product_na[i] = rtb_Sum_a_0[i];
  }

  /* End of Sum: '<S10>/Sum' */
  for (i = 0; i < 3; i++) {
    /* Product: '<S66>/Product' incorporates:
     *  Product: '<S23>/F stab to body'
     */
    rtb_Sum_gb[i] = rtb_referencearea * (rtb_DCM_bs[i + 6] * rtb_Product_na[2] +
      (rtb_DCM_bs[i + 3] * rtb_Product_na[1] + rtb_DCM_bs[i] * rtb_Product_na[0]));
  }

  /* GravityWGS84: '<S154>/WGS84 Gravity Model  ' */
  rtb_elev = Z1_Sim_B.Add[0] * 0.017453292519943295;
  rtb_InterpolationUsingPrelook_m = std::abs(rtb_elev);
  rtb_Product_c = 1.0;
  if (rtb_InterpolationUsingPrelook_m > 3.1415926535897931) {
    if (rtb_elev < -3.1415926535897931) {
      rtb_Product_c = -1.0;
    }

    if (rtIsInf(rtb_InterpolationUsingPrelook_m + 3.1415926535897931)) {
      rtb_UnitConversion_i = (rtNaN);
    } else {
      rtb_UnitConversion_i = std::fmod(rtb_InterpolationUsingPrelook_m +
        3.1415926535897931, 6.2831853071795862);
      rEQ0 = (rtb_UnitConversion_i == 0.0);
      if (!rEQ0) {
        rtb_ail = (rtb_InterpolationUsingPrelook_m + 3.1415926535897931) /
          6.2831853071795862;
        rEQ0 = (std::abs(rtb_ail - std::floor(rtb_ail + 0.5)) <=
                2.2204460492503131E-16 * rtb_ail);
      }

      if (rEQ0) {
        rtb_UnitConversion_i = 0.0;
      }
    }

    rtb_elev = (rtb_UnitConversion_i - 3.1415926535897931) * rtb_Product_c;
    rtb_InterpolationUsingPrelook_m = std::abs(rtb_elev);
  }

  if (rtb_InterpolationUsingPrelook_m > 1.5707963267948966) {
    if (rtb_elev > 1.5707963267948966) {
      rtb_elev = 1.5707963267948966 - (rtb_InterpolationUsingPrelook_m -
        1.5707963267948966);
    }

    if (rtb_elev < -1.5707963267948966) {
      rtb_elev = -(1.5707963267948966 - (rtb_InterpolationUsingPrelook_m -
        1.5707963267948966));
    }
  }

  rtb_f_n = std::sin(rtb_elev);
  rtb_Product_c = rtb_f_n * rtb_f_n;

  /* Gain: '<S12>/Gravity in Earth Axes' incorporates:
   *  GravityWGS84: '<S154>/WGS84 Gravity Model  '
   */
  rtb_ailOut = ((1.0 - (1.006802597171564 - 2.0 * rtb_Product_c / 298.257223563)
                 * 2.0 * Z1_Sim_B.Add[2] / 6.378137E+6) + 3.0 * Z1_Sim_B.Add[2] *
                Z1_Sim_B.Add[2] / 4.0680631590769E+13) * ((0.00193185265241 *
    rtb_Product_c + 1.0) * 9.7803253359 / std::sqrt(1.0 - 0.00669437999014 *
    rtb_Product_c)) * 31.3;

  /* Sum: '<S75>/Sum' incorporates:
   *  Integrator: '<S7>/p,q,r '
   *  Integrator: '<S7>/ub,vb,wb'
   *  Product: '<S111>/i x j'
   *  Product: '<S111>/j x k'
   *  Product: '<S111>/k x i'
   *  Product: '<S112>/i x k'
   *  Product: '<S112>/j x i'
   *  Product: '<S112>/k x j'
   */
  frac_0[0] = Z1_Sim_X.u[1] * Z1_Sim_X.p[2];
  frac_0[1] = Z1_Sim_X.u[2] * Z1_Sim_X.p[0];
  frac_0[2] = Z1_Sim_X.u[0] * Z1_Sim_X.p[1];
  rtb_Product1_pxv[0] = Z1_Sim_X.u[2] * Z1_Sim_X.p[1];
  rtb_Product1_pxv[1] = Z1_Sim_X.u[0] * Z1_Sim_X.p[2];
  rtb_Product1_pxv[2] = Z1_Sim_X.u[1] * Z1_Sim_X.p[0];

  /* Sum: '<S119>/Add' */
  frac_1i[0] = Z1_Sim_B.VectorConcatenate[0];
  frac_1i[1] = Z1_Sim_B.VectorConcatenate[4];
  frac_1i[2] = Z1_Sim_B.VectorConcatenate[8];
  rtb_InterpolationUsingPrelook_m = -0.0;
  for (i = 0; i < 3; i++) {
    /* Sum: '<S7>/Sum' incorporates:
     *  Constant: '<S7>/Constant'
     *  Product: '<S12>/Inertial to Body'
     *  Product: '<S7>/Product'
     *  Sum: '<S6>/Sum'
     *  Sum: '<S75>/Sum'
     */
    Z1_Sim_B.Sum[i] = (rtb_Sum_gb[i] + (Z1_Sim_B.VectorConcatenate[i + 6] *
      rtb_ailOut + (Z1_Sim_B.VectorConcatenate[i + 3] * 0.0 +
                    Z1_Sim_B.VectorConcatenate[i] * 0.0))) / 31.3 + (frac_0[i] -
      rtb_Product1_pxv[i]);

    /* Sum: '<S119>/Add' */
    rtb_InterpolationUsingPrelook_m += frac_1i[i];
  }

  /* If: '<S115>/If' */
  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    rtAction = static_cast<int8_T>(!(rtb_InterpolationUsingPrelook_m > 0.0));
    Z1_Sim_DW.If_ActiveSubsystem = rtAction;
  } else {
    rtAction = Z1_Sim_DW.If_ActiveSubsystem;
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S115>/Positive Trace' incorporates:
     *  ActionPort: '<S117>/Action Port'
     */
    /* Sqrt: '<S117>/sqrt' incorporates:
     *  Constant: '<S117>/Constant'
     *  Sum: '<S117>/Sum'
     */
    rtb_InterpolationUsingPrelook_m = std::sqrt(rtb_InterpolationUsingPrelook_m
      + 1.0);

    /* Gain: '<S117>/Gain' */
    Z1_Sim_B.Merge[0] = 0.5 * rtb_InterpolationUsingPrelook_m;

    /* Gain: '<S117>/Gain1' */
    rtb_InterpolationUsingPrelook_m *= 2.0;

    /* Product: '<S117>/Product' incorporates:
     *  Sum: '<S139>/Add'
     *  Sum: '<S140>/Add'
     *  Sum: '<S141>/Add'
     */
    Z1_Sim_B.Merge[1] = (Z1_Sim_B.VectorConcatenate[7] -
                         Z1_Sim_B.VectorConcatenate[5]) /
      rtb_InterpolationUsingPrelook_m;
    Z1_Sim_B.Merge[2] = (Z1_Sim_B.VectorConcatenate[2] -
                         Z1_Sim_B.VectorConcatenate[6]) /
      rtb_InterpolationUsingPrelook_m;
    Z1_Sim_B.Merge[3] = (Z1_Sim_B.VectorConcatenate[3] -
                         Z1_Sim_B.VectorConcatenate[1]) /
      rtb_InterpolationUsingPrelook_m;

    /* End of Outputs for SubSystem: '<S115>/Positive Trace' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S115>/Negative Trace' incorporates:
     *  ActionPort: '<S116>/Action Port'
     */
    /* If: '<S116>/Find Maximum Diagonal Value' */
    if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
      if ((Z1_Sim_B.VectorConcatenate[4] > Z1_Sim_B.VectorConcatenate[0]) &&
          (Z1_Sim_B.VectorConcatenate[4] > Z1_Sim_B.VectorConcatenate[8])) {
        rtAction = 0;
      } else if (Z1_Sim_B.VectorConcatenate[8] > Z1_Sim_B.VectorConcatenate[0])
      {
        rtAction = 1;
      } else {
        rtAction = 2;
      }

      Z1_Sim_DW.FindMaximumDiagonalValue_Active = rtAction;
    } else {
      rtAction = Z1_Sim_DW.FindMaximumDiagonalValue_Active;
    }

    switch (rtAction) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S116>/Maximum Value at DCM(2,2)' incorporates:
       *  ActionPort: '<S121>/Action Port'
       */
      /* Sqrt: '<S121>/sqrt' incorporates:
       *  Constant: '<S133>/Constant'
       *  Sum: '<S133>/Add'
       */
      rtb_InterpolationUsingPrelook_m = std::sqrt(((Z1_Sim_B.VectorConcatenate[4]
        - Z1_Sim_B.VectorConcatenate[0]) - Z1_Sim_B.VectorConcatenate[8]) + 1.0);

      /* Gain: '<S121>/Gain' */
      Z1_Sim_B.Merge[2] = 0.5 * rtb_InterpolationUsingPrelook_m;

      /* Switch: '<S132>/Switch' incorporates:
       *  Constant: '<S132>/Constant1'
       */
      if (rtb_InterpolationUsingPrelook_m != 0.0) {
        rtb_Saturation = 0.5;
      } else {
        rtb_Saturation = 0.0;
        rtb_InterpolationUsingPrelook_m = 1.0;
      }

      /* End of Switch: '<S132>/Switch' */

      /* Product: '<S132>/Product' */
      rtb_InterpolationUsingPrelook_m = rtb_Saturation /
        rtb_InterpolationUsingPrelook_m;

      /* Gain: '<S121>/Gain1' incorporates:
       *  Product: '<S121>/Product'
       *  Sum: '<S131>/Add'
       */
      Z1_Sim_B.Merge[1] = (Z1_Sim_B.VectorConcatenate[1] +
                           Z1_Sim_B.VectorConcatenate[3]) *
        rtb_InterpolationUsingPrelook_m;

      /* Gain: '<S121>/Gain3' incorporates:
       *  Product: '<S121>/Product'
       *  Sum: '<S130>/Add'
       */
      Z1_Sim_B.Merge[3] = (Z1_Sim_B.VectorConcatenate[5] +
                           Z1_Sim_B.VectorConcatenate[7]) *
        rtb_InterpolationUsingPrelook_m;

      /* Gain: '<S121>/Gain4' incorporates:
       *  Product: '<S121>/Product'
       *  Sum: '<S129>/Add'
       */
      Z1_Sim_B.Merge[0] = (Z1_Sim_B.VectorConcatenate[2] -
                           Z1_Sim_B.VectorConcatenate[6]) *
        rtb_InterpolationUsingPrelook_m;

      /* End of Outputs for SubSystem: '<S116>/Maximum Value at DCM(2,2)' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S116>/Maximum Value at DCM(3,3)' incorporates:
       *  ActionPort: '<S122>/Action Port'
       */
      /* Sqrt: '<S122>/sqrt' incorporates:
       *  Constant: '<S138>/Constant'
       *  Sum: '<S138>/Add'
       */
      rtb_InterpolationUsingPrelook_m = std::sqrt(((Z1_Sim_B.VectorConcatenate[8]
        - Z1_Sim_B.VectorConcatenate[0]) - Z1_Sim_B.VectorConcatenate[4]) + 1.0);

      /* Gain: '<S122>/Gain' */
      Z1_Sim_B.Merge[3] = 0.5 * rtb_InterpolationUsingPrelook_m;

      /* Switch: '<S137>/Switch' incorporates:
       *  Constant: '<S137>/Constant1'
       */
      if (rtb_InterpolationUsingPrelook_m != 0.0) {
        rtb_Saturation = 0.5;
      } else {
        rtb_Saturation = 0.0;
        rtb_InterpolationUsingPrelook_m = 1.0;
      }

      /* End of Switch: '<S137>/Switch' */

      /* Product: '<S137>/Product' */
      rtb_InterpolationUsingPrelook_m = rtb_Saturation /
        rtb_InterpolationUsingPrelook_m;

      /* Gain: '<S122>/Gain1' incorporates:
       *  Product: '<S122>/Product'
       *  Sum: '<S134>/Add'
       */
      Z1_Sim_B.Merge[1] = (Z1_Sim_B.VectorConcatenate[2] +
                           Z1_Sim_B.VectorConcatenate[6]) *
        rtb_InterpolationUsingPrelook_m;

      /* Gain: '<S122>/Gain2' incorporates:
       *  Product: '<S122>/Product'
       *  Sum: '<S135>/Add'
       */
      Z1_Sim_B.Merge[2] = (Z1_Sim_B.VectorConcatenate[5] +
                           Z1_Sim_B.VectorConcatenate[7]) *
        rtb_InterpolationUsingPrelook_m;

      /* Gain: '<S122>/Gain3' incorporates:
       *  Product: '<S122>/Product'
       *  Sum: '<S136>/Add'
       */
      Z1_Sim_B.Merge[0] = (Z1_Sim_B.VectorConcatenate[3] -
                           Z1_Sim_B.VectorConcatenate[1]) *
        rtb_InterpolationUsingPrelook_m;

      /* End of Outputs for SubSystem: '<S116>/Maximum Value at DCM(3,3)' */
      break;

     case 2:
      /* Outputs for IfAction SubSystem: '<S116>/Maximum Value at DCM(1,1)' incorporates:
       *  ActionPort: '<S120>/Action Port'
       */
      /* Sqrt: '<S120>/sqrt' incorporates:
       *  Constant: '<S128>/Constant'
       *  Sum: '<S128>/Add'
       */
      rtb_InterpolationUsingPrelook_m = std::sqrt(((Z1_Sim_B.VectorConcatenate[0]
        - Z1_Sim_B.VectorConcatenate[4]) - Z1_Sim_B.VectorConcatenate[8]) + 1.0);

      /* Gain: '<S120>/Gain' */
      Z1_Sim_B.Merge[1] = 0.5 * rtb_InterpolationUsingPrelook_m;

      /* Switch: '<S127>/Switch' incorporates:
       *  Constant: '<S127>/Constant1'
       */
      if (rtb_InterpolationUsingPrelook_m != 0.0) {
        rtb_Saturation = 0.5;
      } else {
        rtb_Saturation = 0.0;
        rtb_InterpolationUsingPrelook_m = 1.0;
      }

      /* End of Switch: '<S127>/Switch' */

      /* Product: '<S127>/Product' */
      rtb_InterpolationUsingPrelook_m = rtb_Saturation /
        rtb_InterpolationUsingPrelook_m;

      /* Gain: '<S120>/Gain1' incorporates:
       *  Product: '<S120>/Product'
       *  Sum: '<S126>/Add'
       */
      Z1_Sim_B.Merge[2] = (Z1_Sim_B.VectorConcatenate[1] +
                           Z1_Sim_B.VectorConcatenate[3]) *
        rtb_InterpolationUsingPrelook_m;

      /* Gain: '<S120>/Gain2' incorporates:
       *  Product: '<S120>/Product'
       *  Sum: '<S124>/Add'
       */
      Z1_Sim_B.Merge[3] = (Z1_Sim_B.VectorConcatenate[2] +
                           Z1_Sim_B.VectorConcatenate[6]) *
        rtb_InterpolationUsingPrelook_m;

      /* Gain: '<S120>/Gain3' incorporates:
       *  Product: '<S120>/Product'
       *  Sum: '<S125>/Add'
       */
      Z1_Sim_B.Merge[0] = (Z1_Sim_B.VectorConcatenate[7] -
                           Z1_Sim_B.VectorConcatenate[5]) *
        rtb_InterpolationUsingPrelook_m;

      /* End of Outputs for SubSystem: '<S116>/Maximum Value at DCM(1,1)' */
      break;
    }

    /* End of If: '<S116>/Find Maximum Diagonal Value' */
    /* End of Outputs for SubSystem: '<S115>/Negative Trace' */
    break;
  }

  /* End of If: '<S115>/If' */

  /* DataStoreWrite: '<S114>/Data Store Write' incorporates:
   *  BusCreator: '<S114>/Bus Creator1'
   *  Integrator: '<S114>/Integrator'
   *  Integrator: '<S7>/p,q,r '
   *  Integrator: '<S7>/xe,ye,ze'
   */
  Z1_Sim_DW.ACBus_o.time = Z1_Sim_X.Integrator_CSTATE;
  Z1_Sim_DW.ACBus_o.Wb[0] = Z1_Sim_X.p[0];
  Z1_Sim_DW.ACBus_o.Ve[0] = Z1_Sim_B.Product[0];
  Z1_Sim_DW.ACBus_o.Ab[0] = Z1_Sim_B.Sum[0];
  Z1_Sim_DW.ACBus_o.Wb[1] = Z1_Sim_X.p[1];
  Z1_Sim_DW.ACBus_o.Ve[1] = Z1_Sim_B.Product[1];
  Z1_Sim_DW.ACBus_o.Ab[1] = Z1_Sim_B.Sum[1];
  Z1_Sim_DW.ACBus_o.Wb[2] = Z1_Sim_X.p[2];
  Z1_Sim_DW.ACBus_o.Ve[2] = Z1_Sim_B.Product[2];
  Z1_Sim_DW.ACBus_o.Ab[2] = Z1_Sim_B.Sum[2];
  Z1_Sim_DW.ACBus_o.quat[0] = Z1_Sim_B.Merge[0];
  Z1_Sim_DW.ACBus_o.quat[1] = Z1_Sim_B.Merge[1];
  Z1_Sim_DW.ACBus_o.quat[2] = Z1_Sim_B.Merge[2];
  Z1_Sim_DW.ACBus_o.quat[3] = Z1_Sim_B.Merge[3];
  Z1_Sim_DW.ACBus_o.Xe[0] = Z1_Sim_X.Xe[0];
  Z1_Sim_DW.ACBus_o.Xe[1] = Z1_Sim_X.Xe[1];
  Z1_Sim_DW.ACBus_o.Xe[2] = Z1_Sim_X.Xe[2];

  /* If: '<S118>/If1' */
  if ((rtmIsMajorTimeStep((&Z1_Sim_M)) &&
       (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) && rtmIsMajorTimeStep
      ((&Z1_Sim_M))) {
    Z1_Sim_DW.If1_ActiveSubsystem = -1;
  }

  /* Sum: '<S67>/Sum' incorporates:
   *  Product: '<S71>/i x j'
   *  Product: '<S71>/j x k'
   *  Product: '<S71>/k x i'
   *  Product: '<S72>/i x k'
   *  Product: '<S72>/j x i'
   *  Product: '<S72>/k x j'
   */
  rtb_Sum2_m[0] = rtb_Sum_gb[1] * Z1_Sim_ConstB.Sum[2];
  rtb_Sum2_m[1] = rtb_Sum_gb[2] * Z1_Sim_ConstB.Sum[0];
  rtb_Sum2_m[2] = rtb_Sum_gb[0] * Z1_Sim_ConstB.Sum[1];
  frac_1[0] = rtb_Sum_gb[2] * Z1_Sim_ConstB.Sum[1];
  frac_1[1] = rtb_Sum_gb[0] * Z1_Sim_ConstB.Sum[2];
  frac_1[2] = rtb_Sum_gb[1] * Z1_Sim_ConstB.Sum[0];

  /* Product: '<S66>/Product1' incorporates:
   *  Constant: '<S66>/Constant'
   *  Constant: '<S66>/Constant1'
   */
  frac_0[0] = 10.0 * rtb_referencearea;
  frac_0[1] = 0.4 * rtb_referencearea;
  frac_0[2] = 10.0 * rtb_referencearea;
  for (i = 0; i < 3; i++) {
    /* Sum: '<S66>/Sum1' incorporates:
     *  Product: '<S23>/M stab to body'
     *  Product: '<S66>/Product3'
     *  Sum: '<S67>/Sum'
     */
    frac_1i[i] = (rtb_Sum2_m[i] - frac_1[i]) + (rtb_DCM_bs[i + 6] *
      rtb_Product_na[5] + (rtb_DCM_bs[i + 3] * rtb_Product_na[4] + rtb_DCM_bs[i]
      * rtb_Product_na[3])) * frac_0[i];

    /* Product: '<S81>/Product' incorporates:
     *  Integrator: '<S7>/p,q,r '
     */
    rtb_Sum_gb[i] = Z1_Sim_ConstB.Selector[i + 6] * Z1_Sim_X.p[2] +
      (Z1_Sim_ConstB.Selector[i + 3] * Z1_Sim_X.p[1] + Z1_Sim_ConstB.Selector[i]
       * Z1_Sim_X.p[0]);

    /* Product: '<S82>/Product' incorporates:
     *  Integrator: '<S7>/p,q,r '
     */
    rtb_Product1_pxv[i] = 0.0 * Z1_Sim_X.p[2] + (0.0 * Z1_Sim_X.p[1] + 0.0 *
      Z1_Sim_X.p[0]);
  }

  /* Sum: '<S73>/Sum2' incorporates:
   *  Integrator: '<S7>/p,q,r '
   *  Product: '<S82>/Product'
   *  Product: '<S83>/i x j'
   *  Product: '<S83>/j x k'
   *  Product: '<S83>/k x i'
   *  Product: '<S84>/i x k'
   *  Product: '<S84>/j x i'
   *  Product: '<S84>/k x j'
   *  Sum: '<S80>/Sum'
   */
  frac_0[0] = (frac_1i[0] - rtb_Product1_pxv[0]) - (Z1_Sim_X.p[1] * rtb_Sum_gb[2]
    - Z1_Sim_X.p[2] * rtb_Sum_gb[1]);
  frac_0[1] = (frac_1i[1] - rtb_Product1_pxv[1]) - (Z1_Sim_X.p[2] * rtb_Sum_gb[0]
    - Z1_Sim_X.p[0] * rtb_Sum_gb[2]);
  frac_0[2] = (frac_1i[2] - rtb_Product1_pxv[2]) - (Z1_Sim_X.p[0] * rtb_Sum_gb[1]
    - Z1_Sim_X.p[1] * rtb_Sum_gb[0]);

  /* Product: '<S73>/Product2' */
  rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(frac_0, Z1_Sim_ConstB.Selector2,
    Z1_Sim_B.Product2);

  /* Fcn: '<S87>/fcn3' incorporates:
   *  Integrator: '<S85>/q0 q1 q2 q3'
   *  Product: '<S104>/Product'
   *  Product: '<S104>/Product1'
   */
  rtb_fcn3 = (Z1_Sim_X.qr[1] / rtb_fcn3 * rtb_f_f_tmp - Z1_Sim_X.qr[0] /
              rtb_fcn3 * rtb_Saturation_tmp) * -2.0;

  /* If: '<S105>/If' incorporates:
   *  Constant: '<S106>/Constant'
   *  Constant: '<S107>/Constant'
   *  Inport: '<S108>/In'
   */
  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    if (rtb_fcn3 > 1.0) {
      rtAction = 0;
    } else if (rtb_fcn3 < -1.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    Z1_Sim_DW.If_ActiveSubsystem_k = rtAction;
  } else {
    rtAction = Z1_Sim_DW.If_ActiveSubsystem_k;
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S105>/If Action Subsystem' incorporates:
     *  ActionPort: '<S106>/Action Port'
     */
    if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
        (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
      Z1_Sim_B.Merge_k = 1.0;
    }

    /* End of Outputs for SubSystem: '<S105>/If Action Subsystem' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S105>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S107>/Action Port'
     */
    if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
        (&Z1_Sim_M)->Timing.TaskCounters.TID[1] == 0) {
      Z1_Sim_B.Merge_k = 1.0;
    }

    /* End of Outputs for SubSystem: '<S105>/If Action Subsystem1' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S105>/If Action Subsystem2' incorporates:
     *  ActionPort: '<S108>/Action Port'
     */
    Z1_Sim_B.Merge_k = rtb_fcn3;

    /* End of Outputs for SubSystem: '<S105>/If Action Subsystem2' */
    break;
  }

  /* End of If: '<S105>/If' */

  /* Gain: '<S89>/High Gain Quaternion Normalization' incorporates:
   *  Constant: '<S89>/Constant'
   *  DotProduct: '<S89>/Dot Product'
   *  Integrator: '<S85>/q0 q1 q2 q3'
   *  Sum: '<S89>/Sum'
   */
  rtb_HighGainQuaternionNormaliza = (1.0 - (((Z1_Sim_X.qr[0] * Z1_Sim_X.qr[0] +
    Z1_Sim_X.qr[1] * Z1_Sim_X.qr[1]) + Z1_Sim_X.qr[2] * Z1_Sim_X.qr[2]) +
    Z1_Sim_X.qr[3] * Z1_Sim_X.qr[3])) * 0.0;

  /* SignalConversion generated from: '<S85>/q0 q1 q2 q3' incorporates:
   *  Fcn: '<S89>/q0dot'
   *  Fcn: '<S89>/q1dot'
   *  Fcn: '<S89>/q2dot'
   *  Fcn: '<S89>/q3dot'
   *  Integrator: '<S7>/p,q,r '
   *  Integrator: '<S85>/q0 q1 q2 q3'
   */
  Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[0] = ((Z1_Sim_X.qr[1] * Z1_Sim_X.p[0]
    + Z1_Sim_X.qr[2] * Z1_Sim_X.p[1]) + Z1_Sim_X.qr[3] * Z1_Sim_X.p[2]) * -0.5 +
    rtb_HighGainQuaternionNormaliza * Z1_Sim_X.qr[0];
  Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[1] = ((Z1_Sim_X.qr[0] * Z1_Sim_X.p[0]
    + Z1_Sim_X.qr[2] * Z1_Sim_X.p[2]) - Z1_Sim_X.qr[3] * Z1_Sim_X.p[1]) * 0.5 +
    rtb_HighGainQuaternionNormaliza * Z1_Sim_X.qr[1];
  Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[2] = ((Z1_Sim_X.qr[0] * Z1_Sim_X.p[1]
    + Z1_Sim_X.qr[3] * Z1_Sim_X.p[0]) - Z1_Sim_X.qr[1] * Z1_Sim_X.p[2]) * 0.5 +
    rtb_HighGainQuaternionNormaliza * Z1_Sim_X.qr[2];
  Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[3] = ((Z1_Sim_X.qr[0] * Z1_Sim_X.p[2]
    + Z1_Sim_X.qr[1] * Z1_Sim_X.p[1]) - Z1_Sim_X.qr[2] * Z1_Sim_X.p[0]) * 0.5 +
    rtb_HighGainQuaternionNormaliza * Z1_Sim_X.qr[3];
  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    /* Update for Integrator: '<S85>/q0 q1 q2 q3' */
    Z1_Sim_DW.q0q1q2q3_IWORK = 0;
    if (rtmIsMajorTimeStep((&Z1_Sim_M)) &&
        (&Z1_Sim_M)->Timing.TaskCounters.TID[2] == 0) {
      /* Update for RandomNumber: '<S181>/White Noise' */
      Z1_Sim_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf(&Z1_Sim_DW.RandSeed[0]);
      Z1_Sim_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf(&Z1_Sim_DW.RandSeed[1]);
      Z1_Sim_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf(&Z1_Sim_DW.RandSeed[2]);
      Z1_Sim_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf(&Z1_Sim_DW.RandSeed[3]);
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&Z1_Sim_M))) {
    rt_ertODEUpdateContinuousStates(&(&Z1_Sim_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&Z1_Sim_M)->Timing.clockTick0)) {
      ++(&Z1_Sim_M)->Timing.clockTickH0;
    }

    (&Z1_Sim_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&Z1_Sim_M)->solverInfo);

    {
      /* Update absolute timer for sample time: [0.033333333333333333s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.033333333333333333, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&Z1_Sim_M)->Timing.clockTick1++;
      if (!(&Z1_Sim_M)->Timing.clockTick1) {
        (&Z1_Sim_M)->Timing.clockTickH1++;
      }
    }

    rate_scheduler((&Z1_Sim_M));
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void Z1_SimModelClass::Z1_Sim_derivatives()
{
  boolean_T lsat;
  boolean_T usat;
  XDot_Z1_Sim_T *_rtXdot;
  _rtXdot = ((XDot_Z1_Sim_T *) (&Z1_Sim_M)->derivs);

  /* Derivatives for Integrator: '<S114>/Integrator' incorporates:
   *  Constant: '<S114>/Constant'
   */
  _rtXdot->Integrator_CSTATE = 1.0;

  /* Derivatives for Integrator: '<S7>/p,q,r ' */
  if (Z1_Sim_B.TmpSignalConversionAtpqrInport2[0] == 0.0) {
    _rtXdot->p[0] = Z1_Sim_B.Product2[0];
  } else {
    /* level reset is active */
    _rtXdot->p[0] = 0.0;
  }

  if (Z1_Sim_B.TmpSignalConversionAtpqrInport2[1] == 0.0) {
    _rtXdot->p[1] = Z1_Sim_B.Product2[1];
  } else {
    /* level reset is active */
    _rtXdot->p[1] = 0.0;
  }

  if (Z1_Sim_B.TmpSignalConversionAtpqrInport2[2] == 0.0) {
    _rtXdot->p[2] = Z1_Sim_B.Product2[2];
  } else {
    /* level reset is active */
    _rtXdot->p[2] = 0.0;
  }

  /* End of Derivatives for Integrator: '<S7>/p,q,r ' */

  /* Derivatives for Integrator: '<S85>/q0 q1 q2 q3' */
  _rtXdot->qr[0] = Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[0];
  _rtXdot->qr[1] = Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[1];
  _rtXdot->qr[2] = Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[2];
  _rtXdot->qr[3] = Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[3];

  /* Derivatives for Integrator: '<S7>/ub,vb,wb' incorporates:
   *  Integrator: '<S7>/xe,ye,ze'
   */
  if (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[0] == 0.0) {
    _rtXdot->u[0] = Z1_Sim_B.Sum[0];
    _rtXdot->Xe[0] = Z1_Sim_B.Product[0];
  } else {
    /* level reset is active */
    _rtXdot->u[0] = 0.0;

    /* level reset is active */
    _rtXdot->Xe[0] = 0.0;
  }

  if (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[1] == 0.0) {
    _rtXdot->u[1] = Z1_Sim_B.Sum[1];
    _rtXdot->Xe[1] = Z1_Sim_B.Product[1];
  } else {
    /* level reset is active */
    _rtXdot->u[1] = 0.0;

    /* level reset is active */
    _rtXdot->Xe[1] = 0.0;
  }

  if (Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[2] == 0.0) {
    _rtXdot->u[2] = Z1_Sim_B.Sum[2];
    _rtXdot->Xe[2] = Z1_Sim_B.Product[2];
  } else {
    /* level reset is active */
    _rtXdot->u[2] = 0.0;

    /* level reset is active */
    _rtXdot->Xe[2] = 0.0;
  }

  /* End of Derivatives for Integrator: '<S7>/ub,vb,wb' */

  /* Derivatives for Enabled SubSystem: '<S172>/Hugw(s)' */
  if (Z1_Sim_DW.Hugws_MODE) {
    /* Derivatives for Integrator: '<S185>/ug_p' */
    _rtXdot->ug_p_CSTATE[0] = Z1_Sim_B.w_e[0];
    _rtXdot->ug_p_CSTATE[1] = Z1_Sim_B.w_e[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Z1_Sim_T *) (&Z1_Sim_M)->derivs)->ug_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S172>/Hugw(s)' */

  /* Derivatives for Enabled SubSystem: '<S172>/Hvgw(s)' */
  if (Z1_Sim_DW.Hvgws_MODE) {
    /* Derivatives for Integrator: '<S186>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[0] = Z1_Sim_B.w_h[0];

    /* Derivatives for Integrator: '<S186>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[0] = Z1_Sim_B.w_j[0];

    /* Derivatives for Integrator: '<S186>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[1] = Z1_Sim_B.w_h[1];

    /* Derivatives for Integrator: '<S186>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[1] = Z1_Sim_B.w_j[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Z1_Sim_T *) (&Z1_Sim_M)->derivs)->vg_p1_CSTATE[0]);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S172>/Hvgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S172>/Hwgw(s)' */
  if (Z1_Sim_DW.Hwgws_MODE) {
    /* Derivatives for Integrator: '<S187>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[0] = Z1_Sim_B.w[0];

    /* Derivatives for Integrator: '<S187>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[0] = Z1_Sim_B.w_k[0];

    /* Derivatives for Integrator: '<S187>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[1] = Z1_Sim_B.w[1];

    /* Derivatives for Integrator: '<S187>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[1] = Z1_Sim_B.w_k[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Z1_Sim_T *) (&Z1_Sim_M)->derivs)->wg_p1_CSTATE[0]);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S172>/Hwgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S163>/Distance into gust (x)' */
  if (Z1_Sim_DW.Distanceintogustx_MODE) {
    /* Derivatives for Integrator: '<S166>/Distance into Gust (x) (Limited to gust length d)' */
    lsat = (Z1_Sim_X.DistanceintoGustxLimitedtogus_n <= 0.0);
    usat = (Z1_Sim_X.DistanceintoGustxLimitedtogus_n >= 120.0);
    if (((!lsat) && (!usat)) || (lsat && (Z1_Sim_B.Sqrt > 0.0)) || (usat &&
         (Z1_Sim_B.Sqrt < 0.0))) {
      _rtXdot->DistanceintoGustxLimitedtogus_n = Z1_Sim_B.Sqrt;
    } else {
      /* in saturation */
      _rtXdot->DistanceintoGustxLimitedtogus_n = 0.0;
    }

    /* End of Derivatives for Integrator: '<S166>/Distance into Gust (x) (Limited to gust length d)' */
  } else {
    ((XDot_Z1_Sim_T *) (&Z1_Sim_M)->derivs)->DistanceintoGustxLimitedtogus_n =
      0.0;
  }

  /* End of Derivatives for SubSystem: '<S163>/Distance into gust (x)' */

  /* Derivatives for Enabled SubSystem: '<S163>/Distance into gust (y)' */
  Z1_Sim_Distanceintogusty_Deriv(Z1_Sim_B.Sqrt, &Z1_Sim_DW.Distanceintogusty,
    &Z1_Sim_X.Distanceintogusty, &_rtXdot->Distanceintogusty, 120.0);

  /* End of Derivatives for SubSystem: '<S163>/Distance into gust (y)' */

  /* Derivatives for Enabled SubSystem: '<S163>/Distance into gust (z)' */
  Z1_Sim_Distanceintogusty_Deriv(Z1_Sim_B.Sqrt, &Z1_Sim_DW.Distanceintogustz,
    &Z1_Sim_X.Distanceintogustz, &_rtXdot->Distanceintogustz, 80.0);

  /* End of Derivatives for SubSystem: '<S163>/Distance into gust (z)' */

  /* Derivatives for Enabled SubSystem: '<S171>/Hpgw' */
  if (Z1_Sim_DW.Hpgw_MODE) {
    /* Derivatives for Integrator: '<S182>/pgw_p' */
    _rtXdot->pgw_p_CSTATE[0] = Z1_Sim_B.w_hh[0];
    _rtXdot->pgw_p_CSTATE[1] = Z1_Sim_B.w_hh[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Z1_Sim_T *) (&Z1_Sim_M)->derivs)->pgw_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S171>/Hpgw' */

  /* Derivatives for Enabled SubSystem: '<S171>/Hqgw' */
  if (Z1_Sim_DW.Hqgw_MODE) {
    /* Derivatives for Integrator: '<S183>/qgw_p' */
    _rtXdot->qgw_p_CSTATE[0] = Z1_Sim_B.w_b[0];
    _rtXdot->qgw_p_CSTATE[1] = Z1_Sim_B.w_b[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Z1_Sim_T *) (&Z1_Sim_M)->derivs)->qgw_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S171>/Hqgw' */

  /* Derivatives for Enabled SubSystem: '<S171>/Hrgw' */
  if (Z1_Sim_DW.Hrgw_MODE) {
    /* Derivatives for Integrator: '<S184>/rgw_p' */
    _rtXdot->rgw_p_CSTATE[0] = Z1_Sim_B.w_jk[0];
    _rtXdot->rgw_p_CSTATE[1] = Z1_Sim_B.w_jk[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_Z1_Sim_T *) (&Z1_Sim_M)->derivs)->rgw_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S171>/Hrgw' */
}

/* Model initialize function */
void Z1_SimModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&Z1_Sim_M)->solverInfo, &(&Z1_Sim_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&Z1_Sim_M)->solverInfo, &rtmGetTPtr((&Z1_Sim_M)));
    rtsiSetStepSizePtr(&(&Z1_Sim_M)->solverInfo, &(&Z1_Sim_M)->Timing.stepSize0);
    rtsiSetdXPtr(&(&Z1_Sim_M)->solverInfo, &(&Z1_Sim_M)->derivs);
    rtsiSetContStatesPtr(&(&Z1_Sim_M)->solverInfo, (real_T **) &(&Z1_Sim_M)
                         ->contStates);
    rtsiSetNumContStatesPtr(&(&Z1_Sim_M)->solverInfo, &(&Z1_Sim_M)
      ->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&Z1_Sim_M)->solverInfo, &(&Z1_Sim_M)
      ->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&Z1_Sim_M)->solverInfo, &(&Z1_Sim_M)
      ->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&Z1_Sim_M)->solverInfo, &(&Z1_Sim_M)
      ->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&Z1_Sim_M)->solverInfo, (&rtmGetErrorStatus
      ((&Z1_Sim_M))));
    rtsiSetRTModelPtr(&(&Z1_Sim_M)->solverInfo, (&Z1_Sim_M));
  }

  rtsiSetSimTimeStep(&(&Z1_Sim_M)->solverInfo, MAJOR_TIME_STEP);
  (&Z1_Sim_M)->intgData.f[0] = (&Z1_Sim_M)->odeF[0];
  (&Z1_Sim_M)->contStates = ((X_Z1_Sim_T *) &Z1_Sim_X);
  rtsiSetSolverData(&(&Z1_Sim_M)->solverInfo, static_cast<void *>(&(&Z1_Sim_M)
    ->intgData));
  rtsiSetSolverName(&(&Z1_Sim_M)->solverInfo,"ode1");
  rtmSetTPtr((&Z1_Sim_M), &(&Z1_Sim_M)->Timing.tArray[0]);
  (&Z1_Sim_M)->Timing.stepSize0 = 0.033333333333333333;
  rtmSetFirstInitCond((&Z1_Sim_M), 1);

  /* block I/O */
  (void) std::memset((static_cast<void *>(&Z1_Sim_B)), 0,
                     sizeof(B_Z1_Sim_T));

  {
    int32_T i;
    for (i = 0; i < 9; i++) {
      Z1_Sim_B.VectorConcatenate[i] = 0.0;
    }

    Z1_Sim_B.DataStoreRead = Z1_Sim_rtZCmdBus;
    Z1_Sim_B.TmpSignalConversionAtpqrInport2[0] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtpqrInport2[1] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtpqrInport2[2] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[0] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[1] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[2] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtq0q1q2q3In[3] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[0] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[1] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtubvbwbInpo[2] = 0.0;
    Z1_Sim_B.Product[0] = 0.0;
    Z1_Sim_B.Product[1] = 0.0;
    Z1_Sim_B.Product[2] = 0.0;
    Z1_Sim_B.Add[0] = 0.0;
    Z1_Sim_B.Add[1] = 0.0;
    Z1_Sim_B.Add[2] = 0.0;
    Z1_Sim_B.SFunction_o1 = 0.0;
    Z1_Sim_B.SFunction_o2 = 0.0;
    Z1_Sim_B.SFunction_o3 = 0.0;
    Z1_Sim_B.SFunction_o4 = 0.0;
    Z1_Sim_B.Sqrt = 0.0;
    Z1_Sim_B.Product_j[0] = 0.0;
    Z1_Sim_B.Product_j[1] = 0.0;
    Z1_Sim_B.Product_j[2] = 0.0;
    Z1_Sim_B.Product_j[3] = 0.0;
    Z1_Sim_B.Gain1 = 0.0;
    Z1_Sim_B.Gain1_n = 0.0;
    Z1_Sim_B.Gain1_j = 0.0;
    Z1_Sim_B.Gain1_nt = 0.0;
    Z1_Sim_B.Gain1_h = 0.0;
    Z1_Sim_B.Sum[0] = 0.0;
    Z1_Sim_B.Sum[1] = 0.0;
    Z1_Sim_B.Sum[2] = 0.0;
    Z1_Sim_B.Merge[0] = 0.0;
    Z1_Sim_B.Merge[1] = 0.0;
    Z1_Sim_B.Merge[2] = 0.0;
    Z1_Sim_B.Merge[3] = 0.0;
    Z1_Sim_B.Product2[0] = 0.0;
    Z1_Sim_B.Product2[1] = 0.0;
    Z1_Sim_B.Product2[2] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[0] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[1] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[2] = 0.0;
    Z1_Sim_B.TmpSignalConversionAtq0q1q2q3_d[3] = 0.0;
    Z1_Sim_B.w[0] = 0.0;
    Z1_Sim_B.w[1] = 0.0;
    Z1_Sim_B.LwgV1[0] = 0.0;
    Z1_Sim_B.LwgV1[1] = 0.0;
    Z1_Sim_B.w_k[0] = 0.0;
    Z1_Sim_B.w_k[1] = 0.0;
    Z1_Sim_B.w_h[0] = 0.0;
    Z1_Sim_B.w_h[1] = 0.0;
    Z1_Sim_B.w_j[0] = 0.0;
    Z1_Sim_B.w_j[1] = 0.0;
    Z1_Sim_B.w1[0] = 0.0;
    Z1_Sim_B.w1[1] = 0.0;
    Z1_Sim_B.w_e[0] = 0.0;
    Z1_Sim_B.w_e[1] = 0.0;
    Z1_Sim_B.w1_p[0] = 0.0;
    Z1_Sim_B.w1_p[1] = 0.0;
    Z1_Sim_B.w_jk[0] = 0.0;
    Z1_Sim_B.w_jk[1] = 0.0;
    Z1_Sim_B.UnaryMinus[0] = 0.0;
    Z1_Sim_B.UnaryMinus[1] = 0.0;
    Z1_Sim_B.w_b[0] = 0.0;
    Z1_Sim_B.w_b[1] = 0.0;
    Z1_Sim_B.sigma_w[0] = 0.0;
    Z1_Sim_B.sigma_w[1] = 0.0;
    Z1_Sim_B.w_hh[0] = 0.0;
    Z1_Sim_B.w_hh[1] = 0.0;
    Z1_Sim_B.DistanceintoGustxLimitedtogustl = 0.0;
    Z1_Sim_B.Merge_k = 0.0;
    Z1_Sim_B.Distanceintogustz.DistanceintoGustxLimitedtogustl = 0.0;
    Z1_Sim_B.Distanceintogusty.DistanceintoGustxLimitedtogustl = 0.0;
  }

  /* states (continuous) */
  {
    (void) std::memset(static_cast<void *>(&Z1_Sim_X), 0,
                       sizeof(X_Z1_Sim_T));
  }

  /* states (dwork) */
  (void) std::memset(static_cast<void *>(&Z1_Sim_DW), 0,
                     sizeof(DW_Z1_Sim_T));

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Z1_Sim_DW.SFunction_temp_table[i] = 0.0;
    }
  }

  {
    int32_T i;
    for (i = 0; i < 8; i++) {
      Z1_Sim_DW.SFunction_pres_table[i] = 0.0;
    }
  }

  Z1_Sim_DW.NextOutput[0] = 0.0;
  Z1_Sim_DW.NextOutput[1] = 0.0;
  Z1_Sim_DW.NextOutput[2] = 0.0;
  Z1_Sim_DW.NextOutput[3] = 0.0;

  {
    int32_T i;
    for (i = 0; i < 9; i++) {
      Z1_Sim_DW.Product2_DWORK4[i] = 0.0;
    }
  }

  /* Start for S-Function (saeroatmos): '<S157>/S-Function' */
  {
    real_T *temp_table = (real_T *) &Z1_Sim_DW.SFunction_temp_table[0];
    real_T *pres_table = (real_T *) &Z1_Sim_DW.SFunction_pres_table[0];

    /* COESA */
    /*
     * Initialize COESA pressure and temperature tables.
     */
    InitCalcAtmosCOESA( temp_table, pres_table );
  }

  /* Start for If: '<S177>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  Z1_Sim_DW.ifHeightMaxlowaltitudeelseifHei = -1;

  /* Start for If: '<S176>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  Z1_Sim_DW.ifHeightMaxlowaltitudeelseifH_e = -1;

  /* Start for If: '<S115>/If' */
  Z1_Sim_DW.If_ActiveSubsystem = -1;

  /* Start for IfAction SubSystem: '<S115>/Negative Trace' */
  /* Start for If: '<S116>/Find Maximum Diagonal Value' */
  Z1_Sim_DW.FindMaximumDiagonalValue_Active = -1;

  /* End of Start for SubSystem: '<S115>/Negative Trace' */

  /* Start for If: '<S118>/If1' */
  Z1_Sim_DW.If1_ActiveSubsystem = -1;

  /* Start for DataStoreMemory: '<S114>/Data Store Memory' */
  Z1_Sim_DW.CmdBus_e = Z1_Sim_rtZCmdBus;

  /* Start for DataStoreMemory: '<S114>/Data Store Memory1' */
  Z1_Sim_DW.ACBus_o = Z1_Sim_rtZACBus;

  /* Start for If: '<S105>/If' */
  Z1_Sim_DW.If_ActiveSubsystem_k = -1;
  Z1_Sim_PrevZCX.pqr_Reset_ZCE[0] = UNINITIALIZED_ZCSIG;
  Z1_Sim_PrevZCX.ubvbwb_Reset_ZCE[0] = UNINITIALIZED_ZCSIG;
  Z1_Sim_PrevZCX.xeyeze_Reset_ZCE[0] = UNINITIALIZED_ZCSIG;
  Z1_Sim_PrevZCX.pqr_Reset_ZCE[1] = UNINITIALIZED_ZCSIG;
  Z1_Sim_PrevZCX.ubvbwb_Reset_ZCE[1] = UNINITIALIZED_ZCSIG;
  Z1_Sim_PrevZCX.xeyeze_Reset_ZCE[1] = UNINITIALIZED_ZCSIG;
  Z1_Sim_PrevZCX.pqr_Reset_ZCE[2] = UNINITIALIZED_ZCSIG;
  Z1_Sim_PrevZCX.ubvbwb_Reset_ZCE[2] = UNINITIALIZED_ZCSIG;
  Z1_Sim_PrevZCX.xeyeze_Reset_ZCE[2] = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for Integrator: '<S114>/Integrator' */
  Z1_Sim_X.Integrator_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S7>/p,q,r ' */
  Z1_Sim_X.p[0] = 0.0;
  Z1_Sim_X.p[1] = 0.0;
  Z1_Sim_X.p[2] = 0.0;

  /* InitializeConditions for Integrator: '<S85>/q0 q1 q2 q3' */
  if (rtmIsFirstInitCond((&Z1_Sim_M))) {
    Z1_Sim_X.qr[0] = 0.0;
    Z1_Sim_X.qr[1] = 0.0;
    Z1_Sim_X.qr[2] = 0.0;
    Z1_Sim_X.qr[3] = 0.0;
  }

  Z1_Sim_DW.q0q1q2q3_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S85>/q0 q1 q2 q3' */

  /* InitializeConditions for Integrator: '<S7>/ub,vb,wb' */
  Z1_Sim_X.u[0] = 14.2;

  /* InitializeConditions for Integrator: '<S7>/xe,ye,ze' */
  Z1_Sim_X.Xe[0] = 0.0;

  /* InitializeConditions for Integrator: '<S7>/ub,vb,wb' */
  Z1_Sim_X.u[1] = 0.0;

  /* InitializeConditions for Integrator: '<S7>/xe,ye,ze' */
  Z1_Sim_X.Xe[1] = 0.0;

  /* InitializeConditions for Integrator: '<S7>/ub,vb,wb' */
  Z1_Sim_X.u[2] = 0.3674;

  /* InitializeConditions for Integrator: '<S7>/xe,ye,ze' */
  Z1_Sim_X.Xe[2] = -304.8;

  /* InitializeConditions for RandomNumber: '<S181>/White Noise' */
  Z1_Sim_DW.RandSeed[0] = 1529675776U;
  Z1_Sim_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf(&Z1_Sim_DW.RandSeed[0]);
  Z1_Sim_DW.RandSeed[1] = 1529741312U;
  Z1_Sim_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf(&Z1_Sim_DW.RandSeed[1]);
  Z1_Sim_DW.RandSeed[2] = 1529806848U;
  Z1_Sim_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf(&Z1_Sim_DW.RandSeed[2]);
  Z1_Sim_DW.RandSeed[3] = 1529872384U;
  Z1_Sim_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf(&Z1_Sim_DW.RandSeed[3]);

  /* SystemInitialize for Enabled SubSystem: '<S172>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S185>/ug_p' */
  Z1_Sim_X.ug_p_CSTATE[0] = 0.0;

  /* SystemInitialize for Outport: '<S185>/ugw' */
  Z1_Sim_B.w1_p[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S172>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S172>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S186>/vg_p1' */
  Z1_Sim_X.vg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S186>/vgw_p2' */
  Z1_Sim_X.vgw_p2_CSTATE[0] = 0.0;

  /* SystemInitialize for Outport: '<S186>/vgw' */
  Z1_Sim_B.w1[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S172>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S172>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S187>/wg_p1' */
  Z1_Sim_X.wg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S187>/wg_p2' */
  Z1_Sim_X.wg_p2_CSTATE[0] = 0.0;

  /* SystemInitialize for Outport: '<S187>/wgw' */
  Z1_Sim_B.LwgV1[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S172>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S172>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S185>/ug_p' */
  Z1_Sim_X.ug_p_CSTATE[1] = 0.0;

  /* SystemInitialize for Outport: '<S185>/ugw' */
  Z1_Sim_B.w1_p[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S172>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S172>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S186>/vg_p1' */
  Z1_Sim_X.vg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S186>/vgw_p2' */
  Z1_Sim_X.vgw_p2_CSTATE[1] = 0.0;

  /* SystemInitialize for Outport: '<S186>/vgw' */
  Z1_Sim_B.w1[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S172>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S172>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S187>/wg_p1' */
  Z1_Sim_X.wg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S187>/wg_p2' */
  Z1_Sim_X.wg_p2_CSTATE[1] = 0.0;

  /* SystemInitialize for Outport: '<S187>/wgw' */
  Z1_Sim_B.LwgV1[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S172>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S163>/Distance into gust (x)' */
  /* InitializeConditions for Integrator: '<S166>/Distance into Gust (x) (Limited to gust length d)' */
  Z1_Sim_X.DistanceintoGustxLimitedtogus_n = 0.0;

  /* SystemInitialize for Outport: '<S166>/x' */
  Z1_Sim_B.DistanceintoGustxLimitedtogustl = 0.0;

  /* End of SystemInitialize for SubSystem: '<S163>/Distance into gust (x)' */

  /* SystemInitialize for Enabled SubSystem: '<S163>/Distance into gust (y)' */
  Z1_Sim_Distanceintogusty_Init(&Z1_Sim_B.Distanceintogusty,
    &Z1_Sim_X.Distanceintogusty);

  /* End of SystemInitialize for SubSystem: '<S163>/Distance into gust (y)' */

  /* SystemInitialize for Enabled SubSystem: '<S163>/Distance into gust (z)' */
  Z1_Sim_Distanceintogusty_Init(&Z1_Sim_B.Distanceintogustz,
    &Z1_Sim_X.Distanceintogustz);

  /* End of SystemInitialize for SubSystem: '<S163>/Distance into gust (z)' */

  /* SystemInitialize for Enabled SubSystem: '<S171>/Hpgw' */
  /* InitializeConditions for Integrator: '<S182>/pgw_p' */
  Z1_Sim_X.pgw_p_CSTATE[0] = 0.0;

  /* SystemInitialize for Outport: '<S182>/pgw' */
  Z1_Sim_B.sigma_w[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S171>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S171>/Hqgw' */
  /* InitializeConditions for Integrator: '<S183>/qgw_p' */
  Z1_Sim_X.qgw_p_CSTATE[0] = 0.0;

  /* SystemInitialize for Outport: '<S183>/qgw' */
  Z1_Sim_B.w_b[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S171>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S171>/Hrgw' */
  /* InitializeConditions for Integrator: '<S184>/rgw_p' */
  Z1_Sim_X.rgw_p_CSTATE[0] = 0.0;

  /* SystemInitialize for Outport: '<S184>/rgw' */
  Z1_Sim_B.UnaryMinus[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S171>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S171>/Hpgw' */
  /* InitializeConditions for Integrator: '<S182>/pgw_p' */
  Z1_Sim_X.pgw_p_CSTATE[1] = 0.0;

  /* SystemInitialize for Outport: '<S182>/pgw' */
  Z1_Sim_B.sigma_w[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S171>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S171>/Hqgw' */
  /* InitializeConditions for Integrator: '<S183>/qgw_p' */
  Z1_Sim_X.qgw_p_CSTATE[1] = 0.0;

  /* SystemInitialize for Outport: '<S183>/qgw' */
  Z1_Sim_B.w_b[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S171>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S171>/Hrgw' */
  /* InitializeConditions for Integrator: '<S184>/rgw_p' */
  Z1_Sim_X.rgw_p_CSTATE[1] = 0.0;

  /* SystemInitialize for Outport: '<S184>/rgw' */
  Z1_Sim_B.UnaryMinus[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S171>/Hrgw' */

  /* SystemInitialize for Merge: '<S115>/Merge' */
  Z1_Sim_B.Merge[0] = 1.0;
  Z1_Sim_B.Merge[1] = 0.0;
  Z1_Sim_B.Merge[2] = 0.0;
  Z1_Sim_B.Merge[3] = 0.0;

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond((&Z1_Sim_M))) {
    rtmSetFirstInitCond((&Z1_Sim_M), 0);
  }
}

/* Model terminate function */
void Z1_SimModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
Z1_SimModelClass::Z1_SimModelClass() : Z1_Sim_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
Z1_SimModelClass::~Z1_SimModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_Z1_Sim_T * Z1_SimModelClass::getRTM()
{
  return (&Z1_Sim_M);
}

#pragma GCC diagnostic pop
