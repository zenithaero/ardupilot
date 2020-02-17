
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include "defines.h"

/*
 * Z1_Sim_types.h
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

#ifndef RTW_HEADER_Z1_Sim_types_h_
#define RTW_HEADER_Z1_Sim_types_h_
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

#ifndef DEFINED_TYPEDEF_FOR_struct_Q5g7s8w85ifMicizKPH6w_
#define DEFINED_TYPEDEF_FOR_struct_Q5g7s8w85ifMicizKPH6w_

typedef struct {
  real_T CX0[5766];
  real_T CY0[5766];
  real_T CL0[5766];
  real_T Cl0[5766];
  real_T Cm0[5766];
  real_T Cn0[5766];
  real_T CXp[5766];
  real_T CYp[5766];
  real_T CLp[5766];
  real_T Clp[5766];
  real_T Cmp[5766];
  real_T Cnp[5766];
  real_T CXq[5766];
  real_T CYq[5766];
  real_T CLq[5766];
  real_T Clq[5766];
  real_T Cmq[5766];
  real_T Cnq[5766];
  real_T CXr[5766];
  real_T CYr[5766];
  real_T CLr[5766];
  real_T Clr[5766];
  real_T Cmr[5766];
  real_T Cnr[5766];
  real_T CXb[5766];
  real_T CYb[5766];
  real_T CLb[5766];
  real_T Clb[5766];
  real_T Cmb[5766];
  real_T Cnb[5766];
  real_T CXdF_1[5766];
  real_T CXdF_2[5766];
  real_T CXdF_3[5766];
  real_T CXdF_4[5766];
  real_T CXdF_5[5766];
  real_T CYdF_1[5766];
  real_T CYdF_2[5766];
  real_T CYdF_3[5766];
  real_T CYdF_4[5766];
  real_T CYdF_5[5766];
  real_T CLdF_1[5766];
  real_T CLdF_2[5766];
  real_T CLdF_3[5766];
  real_T CLdF_4[5766];
  real_T CLdF_5[5766];
  real_T CldF_1[5766];
  real_T CldF_2[5766];
  real_T CldF_3[5766];
  real_T CldF_4[5766];
  real_T CldF_5[5766];
  real_T CmdF_1[5766];
  real_T CmdF_2[5766];
  real_T CmdF_3[5766];
  real_T CmdF_4[5766];
  real_T CmdF_5[5766];
  real_T CndF_1[5766];
  real_T CndF_2[5766];
  real_T CndF_3[5766];
  real_T CndF_4[5766];
  real_T CndF_5[5766];
  real_T CXdP_1[5766];
  real_T CXdP_2[5766];
  real_T CYdP_1[5766];
  real_T CYdP_2[5766];
  real_T CLdP_1[5766];
  real_T CLdP_2[5766];
  real_T CldP_1[5766];
  real_T CldP_2[5766];
  real_T CmdP_1[5766];
  real_T CmdP_2[5766];
  real_T CndP_1[5766];
  real_T CndP_2[5766];
} struct_Q5g7s8w85ifMicizKPH6w;

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

#ifndef typedef_sdydNugyMPwWim2QbZMfoSF_Z1_Si_T
#define typedef_sdydNugyMPwWim2QbZMfoSF_Z1_Si_T

typedef struct tag_sdydNugyMPwWim2QbZMfoSF sdydNugyMPwWim2QbZMfoSF_Z1_Si_T;

#endif                               /*typedef_sdydNugyMPwWim2QbZMfoSF_Z1_Si_T*/

/* Forward declaration for rtModel */
typedef struct tag_RTM_Z1_Sim_T RT_MODEL_Z1_Sim_T;

#endif                                 /* RTW_HEADER_Z1_Sim_types_h_ */

#pragma GCC diagnostic pop
