
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include "defines.h"

/*
 * Simulator_types.h
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

#ifndef RTW_HEADER_Simulator_types_h_
#define RTW_HEADER_Simulator_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_CmdBus_
#define DEFINED_TYPEDEF_FOR_CmdBus_

typedef struct {
  real_T thr;
  real_T ail;
  real_T elev;
  real_T rud;
  real_T flaps;
  real_T ailOut;
  real_T thrDiff;
} CmdBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ACBus_
#define DEFINED_TYPEDEF_FOR_ACBus_

typedef struct {
  real_T time;
  real_T Wb[3];
  real_T Ve[3];
  real_T Ab[3];
  real_T quat[4];
  real_T Xe[3];
} ACBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_zneKDO34dO44mjRWmCjwQB_
#define DEFINED_TYPEDEF_FOR_struct_zneKDO34dO44mjRWmCjwQB_

typedef struct {
  real_T CX0[2376];
  real_T CY0[2376];
  real_T CL0[2376];
  real_T Cl0[2376];
  real_T Cm0[2376];
  real_T Cn0[2376];
  real_T CXu[2376];
  real_T CYu[2376];
  real_T CLu[2376];
  real_T Clu[2376];
  real_T Cmu[2376];
  real_T Cnu[2376];
  real_T CXa[2376];
  real_T CYa[2376];
  real_T CLa[2376];
  real_T Cla[2376];
  real_T Cma[2376];
  real_T Cna[2376];
  real_T CXb[2376];
  real_T CYb[2376];
  real_T CLb[2376];
  real_T Clb[2376];
  real_T Cmb[2376];
  real_T Cnb[2376];
  real_T CXp[2376];
  real_T CYp[2376];
  real_T CLp[2376];
  real_T Clp[2376];
  real_T Cmp[2376];
  real_T Cnp[2376];
  real_T CXq[2376];
  real_T CYq[2376];
  real_T CLq[2376];
  real_T Clq[2376];
  real_T Cmq[2376];
  real_T Cnq[2376];
  real_T CXr[2376];
  real_T CYr[2376];
  real_T CLr[2376];
  real_T Clr[2376];
  real_T Cmr[2376];
  real_T Cnr[2376];
  real_T CXdF_1[2376];
  real_T CXdF_2[2376];
  real_T CXdF_3[2376];
  real_T CYdF_1[2376];
  real_T CYdF_2[2376];
  real_T CYdF_3[2376];
  real_T CLdF_1[2376];
  real_T CLdF_2[2376];
  real_T CLdF_3[2376];
  real_T CldF_1[2376];
  real_T CldF_2[2376];
  real_T CldF_3[2376];
  real_T CmdF_1[2376];
  real_T CmdF_2[2376];
  real_T CmdF_3[2376];
  real_T CndF_1[2376];
  real_T CndF_2[2376];
  real_T CndF_3[2376];
  real_T CXdP_1[2376];
  real_T CXdP_2[2376];
  real_T CYdP_1[2376];
  real_T CYdP_2[2376];
  real_T CLdP_1[2376];
  real_T CLdP_2[2376];
  real_T CldP_1[2376];
  real_T CldP_2[2376];
  real_T CmdP_1[2376];
  real_T CmdP_2[2376];
  real_T CndP_1[2376];
  real_T CndP_2[2376];
  real_T CXdF_4[2376];
  real_T CYdF_4[2376];
  real_T CLdF_4[2376];
  real_T CldF_4[2376];
  real_T CmdF_4[2376];
  real_T CndF_4[2376];
  real_T CXdF_5[2376];
  real_T CYdF_5[2376];
  real_T CLdF_5[2376];
  real_T CldF_5[2376];
  real_T CmdF_5[2376];
  real_T CndF_5[2376];
} struct_zneKDO34dO44mjRWmCjwQB;

#endif

#ifndef struct_tag_sdydNugyMPwWim2QbZMfoSF
#define struct_tag_sdydNugyMPwWim2QbZMfoSF

struct tag_sdydNugyMPwWim2QbZMfoSF
{
  real_T a;
  real_T inv_f;
  real_T omega_default;
  real_T GM_default;
  real_T GM_prime;
  real_T omega_prime;
  real_T gamma_e;
  real_T k;
  real_T e2;
  real_T E;
  real_T b;
  real_T b_over_a;
};

#endif                                 /*struct_tag_sdydNugyMPwWim2QbZMfoSF*/

#ifndef typedef_sdydNugyMPwWim2QbZMfoSF_Simul_T
#define typedef_sdydNugyMPwWim2QbZMfoSF_Simul_T

typedef struct tag_sdydNugyMPwWim2QbZMfoSF sdydNugyMPwWim2QbZMfoSF_Simul_T;

#endif                               /*typedef_sdydNugyMPwWim2QbZMfoSF_Simul_T*/

/* Forward declaration for rtModel */
typedef struct tag_RTM_Simulator_T RT_MODEL_Simulator_T;

#endif                                 /* RTW_HEADER_Simulator_types_h_ */

#pragma GCC diagnostic pop
