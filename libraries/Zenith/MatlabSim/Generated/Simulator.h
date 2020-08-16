
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include "defines.h"

/*
 * Simulator.h
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

#ifndef RTW_HEADER_Simulator_h_
#define RTW_HEADER_Simulator_h_
#include <cstring>
#include <cmath>
#include <math.h>
#ifndef Simulator_COMMON_INCLUDES_
# define Simulator_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* Simulator_COMMON_INCLUDES_ */

#include "Simulator_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include "rt_defines.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Zero-crossing (trigger) state for system '<S24>/Triggered Subsystem' */
typedef struct {
  ZCSigState TriggeredSubsystem_Trig_ZCE[3];/* '<S24>/Triggered Subsystem' */
} ZCE_TriggeredSubsystem_Simula_T;

/* Block signals for system '<S23>/NEGATIVE Edge' */
typedef struct {
  boolean_T RelationalOperator1[3];    /* '<S25>/Relational Operator1' */
} B_NEGATIVEEdge_Simulator_T;

/* Block states (default storage) for system '<S23>/NEGATIVE Edge' */
typedef struct {
  boolean_T NEGATIVEEdge_MODE;         /* '<S23>/NEGATIVE Edge' */
} DW_NEGATIVEEdge_Simulator_T;

/* Block signals for system '<S23>/POSITIVE Edge' */
typedef struct {
  boolean_T RelationalOperator1[3];    /* '<S26>/Relational Operator1' */
} B_POSITIVEEdge_Simulator_T;

/* Block states (default storage) for system '<S23>/POSITIVE Edge' */
typedef struct {
  boolean_T POSITIVEEdge_MODE;         /* '<S23>/POSITIVE Edge' */
} DW_POSITIVEEdge_Simulator_T;

/* Block signals for system '<S253>/Distance into gust (y)' */
typedef struct {
  real_T DistanceintoGustxLimitedtogustl;
               /* '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
} B_Distanceintogusty_Simulator_T;

/* Block states (default storage) for system '<S253>/Distance into gust (y)' */
typedef struct {
  boolean_T Distanceintogusty_MODE;    /* '<S253>/Distance into gust (y)' */
} DW_Distanceintogusty_Simulato_T;

/* Continuous states for system '<S253>/Distance into gust (y)' */
typedef struct {
  real_T DistanceintoGustxLimitedtogustl;
               /* '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
} X_Distanceintogusty_Simulator_T;

/* State derivatives for system '<S253>/Distance into gust (y)' */
typedef struct {
  real_T DistanceintoGustxLimitedtogustl;
               /* '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
} XDot_Distanceintogusty_Simula_T;

/* State Disabled for system '<S253>/Distance into gust (y)' */
typedef struct {
  boolean_T DistanceintoGustxLimitedtogustl;
               /* '<S257>/Distance into Gust (x) (Limited to gust length d) ' */
} XDis_Distanceintogusty_Simula_T;

/* Block signals (default storage) */
typedef struct {
  real_T Switch3[3];                   /* '<S55>/Switch3' */
  real_T Switch2[3];                   /* '<S47>/Switch2' */
  real_T Switch8[3];                   /* '<S55>/Switch8' */
  real_T Switch2_a[3];                 /* '<S39>/Switch2' */
  real_T VectorConcatenate[9];         /* '<S36>/Vector Concatenate' */
  real_T Switch1[3];                   /* '<S55>/Switch1' */
  real_T Vx;                           /* '<S8>/cos(phi)' */
  real_T Vy;                           /* '<S8>/sin(phi)' */
  real_T Vz;                           /* '<S8>/Gain' */
  real_T Switch2_n[3];                 /* '<S22>/Switch2' */
  real_T Switch4[3];                   /* '<S55>/Switch4' */
  real_T Switch2_f[3];                 /* '<S62>/Switch2' */
  real_T Add[3];                       /* '<S245>/Add' */
  real_T SFunction_o1;                 /* '<S247>/S-Function' */
  real_T SFunction_o2;                 /* '<S247>/S-Function' */
  real_T SFunction_o3;                 /* '<S247>/S-Function' */
  real_T SFunction_o4;                 /* '<S247>/S-Function' */
  real_T Sqrt;                         /* '<S246>/Sqrt' */
  real_T Product[4];                   /* '<S271>/Product' */
  real_T Constant3;                    /* '<S74>/Constant3' */
  real_T Constant2;                    /* '<S74>/Constant2' */
  real_T Add1[5];                      /* '<S68>/Add1' */
  real_T Constant3_i;                  /* '<S73>/Constant3' */
  real_T Constant2_m;                  /* '<S73>/Constant2' */
  real_T Add_j[2];                     /* '<S68>/Add' */
  real_T Merge[4];                     /* '<S206>/Merge' */
  real_T Clock;                        /* '<S24>/Clock' */
  real_T Switch[3];                    /* '<S20>/Switch' */
  real_T Clock_o;                      /* '<S41>/Clock' */
  real_T Switch_d[3];                  /* '<S37>/Switch' */
  real_T Clock_i;                      /* '<S49>/Clock' */
  real_T Switch_i[3];                  /* '<S45>/Switch' */
  real_T Clock_c;                      /* '<S64>/Clock' */
  real_T Switch_j[3];                  /* '<S60>/Switch' */
  real_T w[2];                         /* '<S277>/w' */
  real_T LwgV1[2];                     /* '<S277>/Lwg//V 1' */
  real_T w_k[2];                       /* '<S277>/w ' */
  real_T w_h[2];                       /* '<S276>/w' */
  real_T w_j[2];                       /* '<S276>/w ' */
  real_T w1[2];                        /* '<S276>/w 1' */
  real_T w_e[2];                       /* '<S275>/w' */
  real_T w1_p[2];                      /* '<S275>/w1' */
  real_T w_jk[2];                      /* '<S274>/w' */
  real_T UnaryMinus[2];                /* '<S274>/Unary Minus' */
  real_T w_b[2];                       /* '<S273>/w' */
  real_T sigma_w[2];                   /* '<S272>/sigma_w' */
  real_T w_hh[2];                      /* '<S272>/w' */
  real_T DistanceintoGustxLimitedtogustl;
                /* '<S256>/Distance into Gust (x) (Limited to gust length d)' */
  boolean_T Memory;                    /* '<S55>/Memory' */
  boolean_T onGround;                  /* '<S55>/Logical Operator1' */
  boolean_T NOT;                       /* '<S8>/NOT' */
  boolean_T LogicalOperator[3];        /* '<S45>/Logical Operator' */
  boolean_T Memory_f[3];               /* '<S48>/Memory' */
  boolean_T LogicalOperator1[3];       /* '<S48>/Logical Operator1' */
  boolean_T LogicalOperator4[3];       /* '<S45>/Logical Operator4' */
  boolean_T LogicalOperator_c[3];      /* '<S37>/Logical Operator' */
  boolean_T Memory_j[3];               /* '<S40>/Memory' */
  boolean_T LogicalOperator1_b[3];     /* '<S40>/Logical Operator1' */
  boolean_T LogicalOperator4_i[3];     /* '<S37>/Logical Operator4' */
  boolean_T LogicalOperator_b[3];      /* '<S20>/Logical Operator' */
  boolean_T Memory_d[3];               /* '<S23>/Memory' */
  boolean_T LogicalOperator1_k[3];     /* '<S23>/Logical Operator1' */
  boolean_T LogicalOperator4_d[3];     /* '<S20>/Logical Operator4' */
  boolean_T LogicalOperator_o[3];      /* '<S60>/Logical Operator' */
  boolean_T Memory_n[3];               /* '<S63>/Memory' */
  boolean_T LogicalOperator1_j[3];     /* '<S63>/Logical Operator1' */
  boolean_T LogicalOperator4_a[3];     /* '<S60>/Logical Operator4' */
  boolean_T LogicalOperator2;          /* '<S253>/Logical Operator2' */
  boolean_T HiddenBuf_InsertedFor_Distancei;/* '<S253>/Logical Operator2' */
  boolean_T LogicalOperator1_o;        /* '<S253>/Logical Operator1' */
  boolean_T HiddenBuf_InsertedFor_Distanc_m;/* '<S253>/Logical Operator1' */
  boolean_T LogicalOperator3;          /* '<S253>/Logical Operator3' */
  boolean_T HiddenBuf_InsertedFor_Distanc_l;/* '<S253>/Logical Operator3' */
  boolean_T NOT_k;                     /* '<S55>/NOT' */
  boolean_T Memory_o;                  /* '<S57>/Memory' */
  boolean_T Logic[2];                  /* '<S57>/Logic' */
  B_Distanceintogusty_Simulator_T Distanceintogustz;/* '<S253>/Distance into gust (z)' */
  B_Distanceintogusty_Simulator_T Distanceintogusty;/* '<S253>/Distance into gust (y)' */
  B_POSITIVEEdge_Simulator_T POSITIVEEdge_l;/* '<S63>/POSITIVE Edge' */
  B_NEGATIVEEdge_Simulator_T NEGATIVEEdge_fy;/* '<S63>/NEGATIVE Edge' */
  B_POSITIVEEdge_Simulator_T POSITIVEEdge_o0;/* '<S48>/POSITIVE Edge' */
  B_NEGATIVEEdge_Simulator_T NEGATIVEEdge_j;/* '<S48>/NEGATIVE Edge' */
  B_POSITIVEEdge_Simulator_T POSITIVEEdge_o;/* '<S40>/POSITIVE Edge' */
  B_NEGATIVEEdge_Simulator_T NEGATIVEEdge_f;/* '<S40>/NEGATIVE Edge' */
  B_POSITIVEEdge_Simulator_T POSITIVEEdge;/* '<S23>/POSITIVE Edge' */
  B_NEGATIVEEdge_Simulator_T NEGATIVEEdge;/* '<S23>/NEGATIVE Edge' */
} B_Simulator_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  ACBus ACBus_o;                       /* '<S205>/Data Store Memory1' */
  CmdBus CmdBus_e;                     /* '<S205>/Data Store Memory' */
  real_T Integrator_DSTATE[5];         /* '<S81>/Integrator' */
  real_T Integrator_DSTATE_m[5];       /* '<S82>/Integrator' */
  real_T Integrator_DSTATE_b[2];       /* '<S77>/Integrator' */
  real_T Integrator_DSTATE_c[2];       /* '<S78>/Integrator' */
  real_T SFunction_temp_table[8];      /* '<S247>/S-Function' */
  real_T SFunction_pres_table[8];      /* '<S247>/S-Function' */
  real_T NextOutput[4];                /* '<S271>/White Noise' */
  real_T Product2_DWORK4[9];           /* '<S11>/Product2' */
  void* Assertion_slioAccessor;        /* '<S238>/Assertion' */
  void* Assertion_slioAccessor_a;      /* '<S239>/Assertion' */
  void* Assertion_slioAccessor_n;      /* '<S240>/Assertion' */
  void* Assertion_slioAccessor_o;      /* '<S241>/Assertion' */
  uint32_T PreLookUpIndexSearchprobofexcee;
                        /* '<S278>/PreLook-Up Index Search  (prob of exceed)' */
  uint32_T PreLookUpIndexSearchaltitude_DW;
                              /* '<S278>/PreLook-Up Index Search  (altitude)' */
  uint32_T RandSeed[4];                /* '<S271>/White Noise' */
  int_T Integrator_IWORK;              /* '<S45>/Integrator' */
  int_T Integrator_IWORK_o;            /* '<S37>/Integrator' */
  int_T Integrator_IWORK_p;            /* '<S20>/Integrator' */
  int_T Integrator_IWORK_c;            /* '<S60>/Integrator' */
  int8_T ifHeightMaxlowaltitudeelseifHei;
  /* '<S267>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  int8_T Integrator_PrevResetState;    /* '<S81>/Integrator' */
  int8_T Integrator_PrevResetState_c;  /* '<S82>/Integrator' */
  int8_T Integrator_PrevResetState_g;  /* '<S77>/Integrator' */
  int8_T Integrator_PrevResetState_l;  /* '<S78>/Integrator' */
  int8_T ifHeightMaxlowaltitudeelseifH_e;
  /* '<S266>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  int8_T If_ActiveSubsystem;           /* '<S206>/If' */
  int8_T If1_ActiveSubsystem;          /* '<S209>/If1' */
  int8_T FindMaximumDiagonalValue_Active;
                                      /* '<S207>/Find Maximum Diagonal Value' */
  boolean_T Memory_PreviousInput;      /* '<S55>/Memory' */
  boolean_T Memory_PreviousInput_f;    /* '<S8>/Memory' */
  boolean_T Memory_PreviousInput_l[3]; /* '<S48>/Memory' */
  boolean_T Memory_PreviousInput_a[3]; /* '<S40>/Memory' */
  boolean_T Memory_PreviousInput_a5[3];/* '<S23>/Memory' */
  boolean_T Memory_PreviousInput_n[3]; /* '<S63>/Memory' */
  boolean_T Memory_PreviousInput_g;    /* '<S57>/Memory' */
  boolean_T Hwgws_MODE;                /* '<S262>/Hwgw(s)' */
  boolean_T Hvgws_MODE;                /* '<S262>/Hvgw(s)' */
  boolean_T Hugws_MODE;                /* '<S262>/Hugw(s)' */
  boolean_T Hrgw_MODE;                 /* '<S261>/Hrgw' */
  boolean_T Hqgw_MODE;                 /* '<S261>/Hqgw' */
  boolean_T Hpgw_MODE;                 /* '<S261>/Hpgw' */
  boolean_T Distanceintogustx_MODE;    /* '<S253>/Distance into gust (x)' */
  DW_Distanceintogusty_Simulato_T Distanceintogustz;/* '<S253>/Distance into gust (z)' */
  DW_Distanceintogusty_Simulato_T Distanceintogusty;/* '<S253>/Distance into gust (y)' */
  DW_POSITIVEEdge_Simulator_T POSITIVEEdge_l;/* '<S63>/POSITIVE Edge' */
  DW_NEGATIVEEdge_Simulator_T NEGATIVEEdge_fy;/* '<S63>/NEGATIVE Edge' */
  DW_POSITIVEEdge_Simulator_T POSITIVEEdge_o0;/* '<S48>/POSITIVE Edge' */
  DW_NEGATIVEEdge_Simulator_T NEGATIVEEdge_j;/* '<S48>/NEGATIVE Edge' */
  DW_POSITIVEEdge_Simulator_T POSITIVEEdge_o;/* '<S40>/POSITIVE Edge' */
  DW_NEGATIVEEdge_Simulator_T NEGATIVEEdge_f;/* '<S40>/NEGATIVE Edge' */
  DW_POSITIVEEdge_Simulator_T POSITIVEEdge;/* '<S23>/POSITIVE Edge' */
  DW_NEGATIVEEdge_Simulator_T NEGATIVEEdge;/* '<S23>/NEGATIVE Edge' */
} DW_Simulator_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S205>/Integrator' */
  real_T Integrator_CSTATE_h;          /* '<S8>/Integrator' */
  real_T Integrator_CSTATE_o[3];       /* '<S45>/Integrator' */
  real_T Integrator_CSTATE_f[3];       /* '<S37>/Integrator' */
  real_T Integrator_CSTATE_b[3];       /* '<S20>/Integrator' */
  real_T Integrator2_CSTATE;           /* '<S8>/Integrator2' */
  real_T Integrator1_CSTATE;           /* '<S8>/Integrator1' */
  real_T Integrator3_CSTATE;           /* '<S8>/Integrator3' */
  real_T Integrator_CSTATE_a[3];       /* '<S60>/Integrator' */
  real_T wg_p1_CSTATE[2];              /* '<S277>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S277>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S276>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S276>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S275>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S274>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S273>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S272>/pgw_p' */
  X_Distanceintogusty_Simulator_T Distanceintogustz;/* '<S253>/Distance into gust (y)' */
  X_Distanceintogusty_Simulator_T Distanceintogusty;/* '<S253>/Distance into gust (y)' */
  real_T DistanceintoGustxLimitedtogus_n;
                /* '<S256>/Distance into Gust (x) (Limited to gust length d)' */
} X_Simulator_T;

/* Periodic continuous state vector (global) */
typedef int_T PeriodicIndX_Simulator_T[3];
typedef real_T PeriodicRngX_Simulator_T[6];

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S205>/Integrator' */
  real_T Integrator_CSTATE_h;          /* '<S8>/Integrator' */
  real_T Integrator_CSTATE_o[3];       /* '<S45>/Integrator' */
  real_T Integrator_CSTATE_f[3];       /* '<S37>/Integrator' */
  real_T Integrator_CSTATE_b[3];       /* '<S20>/Integrator' */
  real_T Integrator2_CSTATE;           /* '<S8>/Integrator2' */
  real_T Integrator1_CSTATE;           /* '<S8>/Integrator1' */
  real_T Integrator3_CSTATE;           /* '<S8>/Integrator3' */
  real_T Integrator_CSTATE_a[3];       /* '<S60>/Integrator' */
  real_T wg_p1_CSTATE[2];              /* '<S277>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S277>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S276>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S276>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S275>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S274>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S273>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S272>/pgw_p' */
  XDot_Distanceintogusty_Simula_T Distanceintogustz;/* '<S253>/Distance into gust (y)' */
  XDot_Distanceintogusty_Simula_T Distanceintogusty;/* '<S253>/Distance into gust (y)' */
  real_T DistanceintoGustxLimitedtogus_n;
                /* '<S256>/Distance into Gust (x) (Limited to gust length d)' */
} XDot_Simulator_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<S205>/Integrator' */
  boolean_T Integrator_CSTATE_h;       /* '<S8>/Integrator' */
  boolean_T Integrator_CSTATE_o[3];    /* '<S45>/Integrator' */
  boolean_T Integrator_CSTATE_f[3];    /* '<S37>/Integrator' */
  boolean_T Integrator_CSTATE_b[3];    /* '<S20>/Integrator' */
  boolean_T Integrator2_CSTATE;        /* '<S8>/Integrator2' */
  boolean_T Integrator1_CSTATE;        /* '<S8>/Integrator1' */
  boolean_T Integrator3_CSTATE;        /* '<S8>/Integrator3' */
  boolean_T Integrator_CSTATE_a[3];    /* '<S60>/Integrator' */
  boolean_T wg_p1_CSTATE[2];           /* '<S277>/wg_p1' */
  boolean_T wg_p2_CSTATE[2];           /* '<S277>/wg_p2' */
  boolean_T vg_p1_CSTATE[2];           /* '<S276>/vg_p1' */
  boolean_T vgw_p2_CSTATE[2];          /* '<S276>/vgw_p2' */
  boolean_T ug_p_CSTATE[2];            /* '<S275>/ug_p' */
  boolean_T rgw_p_CSTATE[2];           /* '<S274>/rgw_p' */
  boolean_T qgw_p_CSTATE[2];           /* '<S273>/qgw_p' */
  boolean_T pgw_p_CSTATE[2];           /* '<S272>/pgw_p' */
  XDis_Distanceintogusty_Simula_T Distanceintogustz;/* '<S253>/Distance into gust (y)' */
  XDis_Distanceintogusty_Simula_T Distanceintogusty;/* '<S253>/Distance into gust (y)' */
  boolean_T DistanceintoGustxLimitedtogus_n;
                /* '<S256>/Distance into Gust (x) (Limited to gust length d)' */
} XDis_Simulator_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Integrator_Reset_ZCE;     /* '<S8>/Integrator' */
  ZCSigState Integrator_Reset_ZCE_m[3];/* '<S45>/Integrator' */
  ZCSigState Integrator_Reset_ZCE_g[3];/* '<S37>/Integrator' */
  ZCSigState Integrator_Reset_ZCE_my[3];/* '<S20>/Integrator' */
  ZCSigState Integrator_Reset_ZCE_c[3];/* '<S60>/Integrator' */
  ZCE_TriggeredSubsystem_Simula_T TriggeredSubsystem_h;/* '<S64>/Triggered Subsystem' */
  ZCE_TriggeredSubsystem_Simula_T TriggeredSubsystem_g;/* '<S49>/Triggered Subsystem' */
  ZCE_TriggeredSubsystem_Simula_T TriggeredSubsystem_m;/* '<S41>/Triggered Subsystem' */
  ZCE_TriggeredSubsystem_Simula_T TriggeredSubsystem;/* '<S24>/Triggered Subsystem' */
} PrevZCX_Simulator_T;

/* Invariant block signals (default storage) */
typedef const struct tag_ConstB_Simulator_T {
  real_T MultiportSwitch[2];           /* '<S23>/Multiport Switch' */
  real_T Selector[9];                  /* '<S11>/Selector' */
  real_T Selector2[9];                 /* '<S11>/Selector2' */
  real_T MultiportSwitch_h[2];         /* '<S40>/Multiport Switch' */
  real_T MultiportSwitch_i[2];         /* '<S48>/Multiport Switch' */
  real_T MultiportSwitch_ib[2];        /* '<S63>/Multiport Switch' */
  real_T Sum[3];                       /* '<S197>/Sum' */
  real_T UnitConversion_cm;            /* '<S298>/Unit Conversion' */
} ConstB_Simulator_T;

#ifndef ODE1_INTG
#define ODE1_INTG

/* ODE1 Integration Data */
typedef struct {
  real_T *f[1];                        /* derivatives */
} ODE1_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S8>/1-D Lookup Table'
   *   '<S23>/neg. edge'
   *   '<S48>/neg. edge'
   *   '<S63>/neg. edge'
   *   '<S40>/neg. edge'
   *   '<S218>/Constant2'
   *   '<S223>/Constant2'
   *   '<S228>/Constant2'
   */
  real_T pooled9[2];

  /* Expression: [0 0 -G]
   * Referenced by: '<S6>/Constant3'
   */
  real_T Constant3_Value[3];

  /* Expression: [0 platform.maxSpd]
   * Referenced by: '<S8>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData[2];

  /* Expression: h_vec
   * Referenced by: '<S278>/PreLook-Up Index Search  (altitude)'
   */
  real_T PreLookUpIndexSearchaltitude_Br[12];

  /* Expression: sigma_vec'
   * Referenced by: '<S278>/Medium//High Altitude Intensity'
   */
  real_T MediumHighAltitudeIntensity_Tab[84];

  /* Pooled Parameter (Expression: aeroData.keys.As)
   * Referenced by:
   *   '<S92>/Interpolation Using Prelookup'
   *   '<S92>/Prelookup'
   */
  real_T pooled28[18];

  /* Expression: aeroData.keys.Vs
   * Referenced by: '<S92>/Prelookup2'
   */
  real_T Prelookup2_1_BreakpointsData[12];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S154>/Interpolation Using Prelookup'
   *   '<S154>/Interpolation Using Prelookup1'
   */
  real_T pooled30[2376];

  /* Expression: aeroData.keys.Bs
   * Referenced by: '<S92>/Prelookup1'
   */
  real_T Prelookup1_1_BreakpointsData[11];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S155>/Interpolation Using Prelookup'
   *   '<S155>/Interpolation Using Prelookup1'
   */
  real_T pooled32[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S153>/Interpolation Using Prelookup'
   *   '<S153>/Interpolation Using Prelookup1'
   */
  real_T pooled33[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S156>/Interpolation Using Prelookup'
   *   '<S156>/Interpolation Using Prelookup1'
   */
  real_T pooled34[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S157>/Interpolation Using Prelookup'
   *   '<S157>/Interpolation Using Prelookup1'
   */
  real_T pooled35[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S158>/Interpolation Using Prelookup'
   *   '<S158>/Interpolation Using Prelookup1'
   */
  real_T pooled36[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S103>/Interpolation Using Prelookup'
   *   '<S103>/Interpolation Using Prelookup1'
   */
  real_T pooled37[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S104>/Interpolation Using Prelookup'
   *   '<S104>/Interpolation Using Prelookup1'
   */
  real_T pooled38[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S102>/Interpolation Using Prelookup'
   *   '<S102>/Interpolation Using Prelookup1'
   */
  real_T pooled39[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S105>/Interpolation Using Prelookup'
   *   '<S105>/Interpolation Using Prelookup1'
   */
  real_T pooled40[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S106>/Interpolation Using Prelookup'
   *   '<S106>/Interpolation Using Prelookup1'
   */
  real_T pooled41[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S107>/Interpolation Using Prelookup'
   *   '<S107>/Interpolation Using Prelookup1'
   */
  real_T pooled42[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S110>/Interpolation Using Prelookup'
   *   '<S110>/Interpolation Using Prelookup1'
   */
  real_T pooled46[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S111>/Interpolation Using Prelookup'
   *   '<S111>/Interpolation Using Prelookup1'
   */
  real_T pooled47[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S109>/Interpolation Using Prelookup'
   *   '<S109>/Interpolation Using Prelookup1'
   */
  real_T pooled48[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S112>/Interpolation Using Prelookup'
   *   '<S112>/Interpolation Using Prelookup1'
   */
  real_T pooled49[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S113>/Interpolation Using Prelookup'
   *   '<S113>/Interpolation Using Prelookup1'
   */
  real_T pooled50[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S114>/Interpolation Using Prelookup'
   *   '<S114>/Interpolation Using Prelookup1'
   */
  real_T pooled51[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S131>/Interpolation Using Prelookup'
   *   '<S131>/Interpolation Using Prelookup1'
   */
  real_T pooled52[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S132>/Interpolation Using Prelookup'
   *   '<S132>/Interpolation Using Prelookup1'
   */
  real_T pooled53[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S130>/Interpolation Using Prelookup'
   *   '<S130>/Interpolation Using Prelookup1'
   */
  real_T pooled54[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S133>/Interpolation Using Prelookup'
   *   '<S133>/Interpolation Using Prelookup1'
   */
  real_T pooled55[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S134>/Interpolation Using Prelookup'
   *   '<S134>/Interpolation Using Prelookup1'
   */
  real_T pooled56[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S135>/Interpolation Using Prelookup'
   *   '<S135>/Interpolation Using Prelookup1'
   */
  real_T pooled57[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S116>/Interpolation Using Prelookup'
   *   '<S116>/Interpolation Using Prelookup1'
   *   '<S117>/Interpolation Using Prelookup'
   *   '<S117>/Interpolation Using Prelookup1'
   *   '<S118>/Interpolation Using Prelookup'
   *   '<S118>/Interpolation Using Prelookup1'
   *   '<S119>/Interpolation Using Prelookup'
   *   '<S119>/Interpolation Using Prelookup1'
   *   '<S120>/Interpolation Using Prelookup'
   *   '<S120>/Interpolation Using Prelookup1'
   *   '<S121>/Interpolation Using Prelookup'
   *   '<S121>/Interpolation Using Prelookup1'
   *   '<S123>/Interpolation Using Prelookup'
   *   '<S123>/Interpolation Using Prelookup1'
   *   '<S124>/Interpolation Using Prelookup'
   *   '<S124>/Interpolation Using Prelookup1'
   *   '<S125>/Interpolation Using Prelookup'
   *   '<S125>/Interpolation Using Prelookup1'
   *   '<S126>/Interpolation Using Prelookup'
   *   '<S126>/Interpolation Using Prelookup1'
   *   '<S127>/Interpolation Using Prelookup'
   *   '<S127>/Interpolation Using Prelookup1'
   *   '<S128>/Interpolation Using Prelookup'
   *   '<S128>/Interpolation Using Prelookup1'
   */
  real_T pooled58[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S140>/Interpolation Using Prelookup'
   *   '<S140>/Interpolation Using Prelookup1'
   */
  real_T pooled59[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S141>/Interpolation Using Prelookup'
   *   '<S141>/Interpolation Using Prelookup1'
   */
  real_T pooled60[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S139>/Interpolation Using Prelookup'
   *   '<S139>/Interpolation Using Prelookup1'
   */
  real_T pooled61[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S142>/Interpolation Using Prelookup'
   *   '<S142>/Interpolation Using Prelookup1'
   */
  real_T pooled62[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S143>/Interpolation Using Prelookup'
   *   '<S143>/Interpolation Using Prelookup1'
   */
  real_T pooled63[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S144>/Interpolation Using Prelookup'
   *   '<S144>/Interpolation Using Prelookup1'
   */
  real_T pooled64[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S147>/Interpolation Using Prelookup'
   *   '<S147>/Interpolation Using Prelookup1'
   */
  real_T pooled65[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S148>/Interpolation Using Prelookup'
   *   '<S148>/Interpolation Using Prelookup1'
   */
  real_T pooled66[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S146>/Interpolation Using Prelookup'
   *   '<S146>/Interpolation Using Prelookup1'
   */
  real_T pooled67[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S149>/Interpolation Using Prelookup'
   *   '<S149>/Interpolation Using Prelookup1'
   */
  real_T pooled68[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S150>/Interpolation Using Prelookup'
   *   '<S150>/Interpolation Using Prelookup1'
   */
  real_T pooled69[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S151>/Interpolation Using Prelookup'
   *   '<S151>/Interpolation Using Prelookup1'
   */
  real_T pooled70[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S164>/Interpolation Using Prelookup'
   *   '<S164>/Interpolation Using Prelookup1'
   */
  real_T pooled71[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S165>/Interpolation Using Prelookup'
   *   '<S165>/Interpolation Using Prelookup1'
   */
  real_T pooled72[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S163>/Interpolation Using Prelookup'
   *   '<S163>/Interpolation Using Prelookup1'
   */
  real_T pooled73[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S166>/Interpolation Using Prelookup'
   *   '<S166>/Interpolation Using Prelookup1'
   */
  real_T pooled74[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S167>/Interpolation Using Prelookup'
   *   '<S167>/Interpolation Using Prelookup1'
   */
  real_T pooled75[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S168>/Interpolation Using Prelookup'
   *   '<S168>/Interpolation Using Prelookup1'
   */
  real_T pooled76[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S171>/Interpolation Using Prelookup'
   *   '<S171>/Interpolation Using Prelookup1'
   */
  real_T pooled78[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S172>/Interpolation Using Prelookup'
   *   '<S172>/Interpolation Using Prelookup1'
   */
  real_T pooled79[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S170>/Interpolation Using Prelookup'
   *   '<S170>/Interpolation Using Prelookup1'
   */
  real_T pooled80[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S173>/Interpolation Using Prelookup'
   *   '<S173>/Interpolation Using Prelookup1'
   */
  real_T pooled81[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S174>/Interpolation Using Prelookup'
   *   '<S174>/Interpolation Using Prelookup1'
   */
  real_T pooled82[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S175>/Interpolation Using Prelookup'
   *   '<S175>/Interpolation Using Prelookup1'
   */
  real_T pooled83[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S178>/Interpolation Using Prelookup'
   *   '<S178>/Interpolation Using Prelookup1'
   */
  real_T pooled84[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S179>/Interpolation Using Prelookup'
   *   '<S179>/Interpolation Using Prelookup1'
   */
  real_T pooled85[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S177>/Interpolation Using Prelookup'
   *   '<S177>/Interpolation Using Prelookup1'
   */
  real_T pooled86[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S180>/Interpolation Using Prelookup'
   *   '<S180>/Interpolation Using Prelookup1'
   */
  real_T pooled87[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S181>/Interpolation Using Prelookup'
   *   '<S181>/Interpolation Using Prelookup1'
   */
  real_T pooled88[2376];

  /* Pooled Parameter (Expression: data.(sprintf('C%s%s', axis, deriv)))
   * Referenced by:
   *   '<S182>/Interpolation Using Prelookup'
   *   '<S182>/Interpolation Using Prelookup1'
   */
  real_T pooled89[2376];

  /* Computed Parameter: MediumHighAltitudeIntensity_max
   * Referenced by: '<S278>/Medium//High Altitude Intensity'
   */
  uint32_T MediumHighAltitudeIntensity_max[2];

  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S153>/Interpolation Using Prelookup'
   *   '<S153>/Interpolation Using Prelookup1'
   *   '<S154>/Interpolation Using Prelookup'
   *   '<S154>/Interpolation Using Prelookup1'
   *   '<S155>/Interpolation Using Prelookup'
   *   '<S155>/Interpolation Using Prelookup1'
   *   '<S156>/Interpolation Using Prelookup'
   *   '<S156>/Interpolation Using Prelookup1'
   *   '<S157>/Interpolation Using Prelookup'
   *   '<S157>/Interpolation Using Prelookup1'
   *   '<S158>/Interpolation Using Prelookup'
   *   '<S158>/Interpolation Using Prelookup1'
   *   '<S163>/Interpolation Using Prelookup'
   *   '<S163>/Interpolation Using Prelookup1'
   *   '<S164>/Interpolation Using Prelookup'
   *   '<S164>/Interpolation Using Prelookup1'
   *   '<S165>/Interpolation Using Prelookup'
   *   '<S165>/Interpolation Using Prelookup1'
   *   '<S166>/Interpolation Using Prelookup'
   *   '<S166>/Interpolation Using Prelookup1'
   *   '<S167>/Interpolation Using Prelookup'
   *   '<S167>/Interpolation Using Prelookup1'
   *   '<S168>/Interpolation Using Prelookup'
   *   '<S168>/Interpolation Using Prelookup1'
   *   '<S170>/Interpolation Using Prelookup'
   *   '<S170>/Interpolation Using Prelookup1'
   *   '<S171>/Interpolation Using Prelookup'
   *   '<S171>/Interpolation Using Prelookup1'
   *   '<S172>/Interpolation Using Prelookup'
   *   '<S172>/Interpolation Using Prelookup1'
   *   '<S173>/Interpolation Using Prelookup'
   *   '<S173>/Interpolation Using Prelookup1'
   *   '<S174>/Interpolation Using Prelookup'
   *   '<S174>/Interpolation Using Prelookup1'
   *   '<S175>/Interpolation Using Prelookup'
   *   '<S175>/Interpolation Using Prelookup1'
   *   '<S177>/Interpolation Using Prelookup'
   *   '<S177>/Interpolation Using Prelookup1'
   *   '<S178>/Interpolation Using Prelookup'
   *   '<S178>/Interpolation Using Prelookup1'
   *   '<S179>/Interpolation Using Prelookup'
   *   '<S179>/Interpolation Using Prelookup1'
   *   '<S180>/Interpolation Using Prelookup'
   *   '<S180>/Interpolation Using Prelookup1'
   *   '<S181>/Interpolation Using Prelookup'
   *   '<S181>/Interpolation Using Prelookup1'
   *   '<S182>/Interpolation Using Prelookup'
   *   '<S182>/Interpolation Using Prelookup1'
   *   '<S102>/Interpolation Using Prelookup'
   *   '<S102>/Interpolation Using Prelookup1'
   *   '<S103>/Interpolation Using Prelookup'
   *   '<S103>/Interpolation Using Prelookup1'
   *   '<S104>/Interpolation Using Prelookup'
   *   '<S104>/Interpolation Using Prelookup1'
   *   '<S105>/Interpolation Using Prelookup'
   *   '<S105>/Interpolation Using Prelookup1'
   *   '<S106>/Interpolation Using Prelookup'
   *   '<S106>/Interpolation Using Prelookup1'
   *   '<S107>/Interpolation Using Prelookup'
   *   '<S107>/Interpolation Using Prelookup1'
   *   '<S109>/Interpolation Using Prelookup'
   *   '<S109>/Interpolation Using Prelookup1'
   *   '<S110>/Interpolation Using Prelookup'
   *   '<S110>/Interpolation Using Prelookup1'
   *   '<S111>/Interpolation Using Prelookup'
   *   '<S111>/Interpolation Using Prelookup1'
   *   '<S112>/Interpolation Using Prelookup'
   *   '<S112>/Interpolation Using Prelookup1'
   *   '<S113>/Interpolation Using Prelookup'
   *   '<S113>/Interpolation Using Prelookup1'
   *   '<S114>/Interpolation Using Prelookup'
   *   '<S114>/Interpolation Using Prelookup1'
   *   '<S116>/Interpolation Using Prelookup'
   *   '<S116>/Interpolation Using Prelookup1'
   *   '<S117>/Interpolation Using Prelookup'
   *   '<S117>/Interpolation Using Prelookup1'
   *   '<S118>/Interpolation Using Prelookup'
   *   '<S118>/Interpolation Using Prelookup1'
   *   '<S119>/Interpolation Using Prelookup'
   *   '<S119>/Interpolation Using Prelookup1'
   *   '<S120>/Interpolation Using Prelookup'
   *   '<S120>/Interpolation Using Prelookup1'
   *   '<S121>/Interpolation Using Prelookup'
   *   '<S121>/Interpolation Using Prelookup1'
   *   '<S123>/Interpolation Using Prelookup'
   *   '<S123>/Interpolation Using Prelookup1'
   *   '<S124>/Interpolation Using Prelookup'
   *   '<S124>/Interpolation Using Prelookup1'
   *   '<S125>/Interpolation Using Prelookup'
   *   '<S125>/Interpolation Using Prelookup1'
   *   '<S126>/Interpolation Using Prelookup'
   *   '<S126>/Interpolation Using Prelookup1'
   *   '<S127>/Interpolation Using Prelookup'
   *   '<S127>/Interpolation Using Prelookup1'
   *   '<S128>/Interpolation Using Prelookup'
   *   '<S128>/Interpolation Using Prelookup1'
   *   '<S130>/Interpolation Using Prelookup'
   *   '<S130>/Interpolation Using Prelookup1'
   *   '<S131>/Interpolation Using Prelookup'
   *   '<S131>/Interpolation Using Prelookup1'
   *   '<S132>/Interpolation Using Prelookup'
   *   '<S132>/Interpolation Using Prelookup1'
   *   '<S133>/Interpolation Using Prelookup'
   *   '<S133>/Interpolation Using Prelookup1'
   *   '<S134>/Interpolation Using Prelookup'
   *   '<S134>/Interpolation Using Prelookup1'
   *   '<S135>/Interpolation Using Prelookup'
   *   '<S135>/Interpolation Using Prelookup1'
   *   '<S139>/Interpolation Using Prelookup'
   *   '<S139>/Interpolation Using Prelookup1'
   *   '<S140>/Interpolation Using Prelookup'
   *   '<S140>/Interpolation Using Prelookup1'
   *   '<S141>/Interpolation Using Prelookup'
   *   '<S141>/Interpolation Using Prelookup1'
   *   '<S142>/Interpolation Using Prelookup'
   *   '<S142>/Interpolation Using Prelookup1'
   *   '<S143>/Interpolation Using Prelookup'
   *   '<S143>/Interpolation Using Prelookup1'
   *   '<S144>/Interpolation Using Prelookup'
   *   '<S144>/Interpolation Using Prelookup1'
   *   '<S146>/Interpolation Using Prelookup'
   *   '<S146>/Interpolation Using Prelookup1'
   *   '<S147>/Interpolation Using Prelookup'
   *   '<S147>/Interpolation Using Prelookup1'
   *   '<S148>/Interpolation Using Prelookup'
   *   '<S148>/Interpolation Using Prelookup1'
   *   '<S149>/Interpolation Using Prelookup'
   *   '<S149>/Interpolation Using Prelookup1'
   *   '<S150>/Interpolation Using Prelookup'
   *   '<S150>/Interpolation Using Prelookup1'
   *   '<S151>/Interpolation Using Prelookup'
   *   '<S151>/Interpolation Using Prelookup1'
   */
  uint32_T pooled91[3];

  /* Computed Parameter: Logic_table
   * Referenced by: '<S57>/Logic'
   */
  boolean_T Logic_table[16];
} ConstP_Simulator_T;

/* Real-time Model Data Structure */
struct tag_RTM_Simulator_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_Simulator_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeF[1][36];
  ODE1_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

/* External data declarations for dependent source files */
extern const CmdBus Simulator_rtZCmdBus;/* CmdBus ground */
extern const ACBus Simulator_rtZACBus; /* ACBus ground */
extern const ConstB_Simulator_T Simulator_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_Simulator_T Simulator_ConstP;

/* Class declaration for model Simulator */
class SimulatorModelClass {
  /* public data and function members */
 public:
DW_Simulator_T Simulator_DW;

  /* model initialize function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  SimulatorModelClass();

  /* Destructor */
  ~SimulatorModelClass();

  /* Real-Time Model get method */
  RT_MODEL_Simulator_T * getRTM();

  /* private data and function members */
 private:
  /* Block signals */
  B_Simulator_T Simulator_B;

  /* Block states */
  //DW_Simulator_T Simulator_DW;
  X_Simulator_T Simulator_X;           /* Block continuous states */
  PeriodicIndX_Simulator_T Simulator_PeriodicIndX;/* Block periodic continuous states */
  PeriodicRngX_Simulator_T Simulator_PeriodicRngX;
  PrevZCX_Simulator_T Simulator_PrevZCX;/* Triggered events */

  /* Real-Time Model */
  RT_MODEL_Simulator_T Simulator_M;

  /* private member function(s) for subsystem '<S24>/Triggered Subsystem'*/
  void Simulator_TriggeredSubsystem(const boolean_T rtu_Trigger[3],
    ZCE_TriggeredSubsystem_Simula_T *localZCE);

  /* private member function(s) for subsystem '<S23>/NEGATIVE Edge'*/
  void Simulator_NEGATIVEEdge_Init(B_NEGATIVEEdge_Simulator_T *localB);
  void Simulator_NEGATIVEEdge_Disable(DW_NEGATIVEEdge_Simulator_T *localDW);
  void Simulator_NEGATIVEEdge(real_T rtu_Enable, const boolean_T rtu_IN[3],
    const boolean_T rtu_INprevious[3], B_NEGATIVEEdge_Simulator_T *localB,
    DW_NEGATIVEEdge_Simulator_T *localDW);

  /* private member function(s) for subsystem '<S23>/POSITIVE Edge'*/
  void Simulator_POSITIVEEdge_Init(B_POSITIVEEdge_Simulator_T *localB);
  void Simulator_POSITIVEEdge_Disable(DW_POSITIVEEdge_Simulator_T *localDW);
  void Simulator_POSITIVEEdge(real_T rtu_Enable, const boolean_T rtu_IN[3],
    const boolean_T rtu_INprevious[3], B_POSITIVEEdge_Simulator_T *localB,
    DW_POSITIVEEdge_Simulator_T *localDW);

  /* private member function(s) for subsystem '<S253>/Distance into gust (y)'*/
  void Simulato_Distanceintogusty_Init(B_Distanceintogusty_Simulator_T *localB,
    X_Distanceintogusty_Simulator_T *localX);
  void Simulat_Distanceintogusty_Reset(X_Distanceintogusty_Simulator_T *localX);
  void Simulat_Distanceintogusty_Deriv(real_T rtu_V,
    DW_Distanceintogusty_Simulato_T *localDW, X_Distanceintogusty_Simulator_T
    *localX, XDot_Distanceintogusty_Simula_T *localXdot, real_T rtp_d_m);
  void Simul_Distanceintogusty_Disable(DW_Distanceintogusty_Simulato_T *localDW);
  void Simulator_Distanceintogusty(boolean_T rtu_Enable,
    B_Distanceintogusty_Simulator_T *localB, DW_Distanceintogusty_Simulato_T
    *localDW, X_Distanceintogusty_Simulator_T *localX, real_T rtp_d_m);

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void Simulator_derivatives();
};

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Simulator'
 * '<S1>'   : 'Simulator/Airframe'
 * '<S2>'   : 'Simulator/Control'
 * '<S3>'   : 'Simulator/Environment'
 * '<S4>'   : 'Simulator/Graph'
 * '<S5>'   : 'Simulator/Output'
 * '<S6>'   : 'Simulator/Airframe/Dynamics'
 * '<S7>'   : 'Simulator/Airframe/F&M'
 * '<S8>'   : 'Simulator/Airframe/Platform model'
 * '<S9>'   : 'Simulator/Airframe/Subsystem1'
 * '<S10>'  : 'Simulator/Airframe/Dynamics/Ab to Vb'
 * '<S11>'  : 'Simulator/Airframe/Dynamics/Calculate omega_dot'
 * '<S12>'  : 'Simulator/Airframe/Dynamics/Euler '
 * '<S13>'  : 'Simulator/Airframe/Dynamics/Rot accel to rot rate'
 * '<S14>'  : 'Simulator/Airframe/Dynamics/State reset logic'
 * '<S15>'  : 'Simulator/Airframe/Dynamics/Vbxw'
 * '<S16>'  : 'Simulator/Airframe/Dynamics/Ve to Xe'
 * '<S17>'  : 'Simulator/Airframe/Dynamics/Velocity Conversion'
 * '<S18>'  : 'Simulator/Airframe/Dynamics/Velocity Conversion1'
 * '<S19>'  : 'Simulator/Airframe/Dynamics/transform to Inertial axes '
 * '<S20>'  : 'Simulator/Airframe/Dynamics/Ab to Vb/Subsystem'
 * '<S21>'  : 'Simulator/Airframe/Dynamics/Ab to Vb/Subsystem/Edge Detector'
 * '<S22>'  : 'Simulator/Airframe/Dynamics/Ab to Vb/Subsystem/Saturation Dynamic'
 * '<S23>'  : 'Simulator/Airframe/Dynamics/Ab to Vb/Subsystem/Edge Detector/Model'
 * '<S24>'  : 'Simulator/Airframe/Dynamics/Ab to Vb/Subsystem/Edge Detector/Model/Internal dirac generator'
 * '<S25>'  : 'Simulator/Airframe/Dynamics/Ab to Vb/Subsystem/Edge Detector/Model/NEGATIVE Edge'
 * '<S26>'  : 'Simulator/Airframe/Dynamics/Ab to Vb/Subsystem/Edge Detector/Model/POSITIVE Edge'
 * '<S27>'  : 'Simulator/Airframe/Dynamics/Ab to Vb/Subsystem/Edge Detector/Model/Internal dirac generator/Triggered Subsystem'
 * '<S28>'  : 'Simulator/Airframe/Dynamics/Calculate omega_dot/3x3 Cross Product'
 * '<S29>'  : 'Simulator/Airframe/Dynamics/Calculate omega_dot/I x w'
 * '<S30>'  : 'Simulator/Airframe/Dynamics/Calculate omega_dot/I x w1'
 * '<S31>'  : 'Simulator/Airframe/Dynamics/Calculate omega_dot/3x3 Cross Product/Subsystem'
 * '<S32>'  : 'Simulator/Airframe/Dynamics/Calculate omega_dot/3x3 Cross Product/Subsystem1'
 * '<S33>'  : 'Simulator/Airframe/Dynamics/Euler /Rotation Angles to Direction Cosine Matrix'
 * '<S34>'  : 'Simulator/Airframe/Dynamics/Euler /phidot thetadot psidot'
 * '<S35>'  : 'Simulator/Airframe/Dynamics/Euler /pqr to Euler'
 * '<S36>'  : 'Simulator/Airframe/Dynamics/Euler /Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S37>'  : 'Simulator/Airframe/Dynamics/Euler /pqr to Euler/Subsystem'
 * '<S38>'  : 'Simulator/Airframe/Dynamics/Euler /pqr to Euler/Subsystem/Edge Detector'
 * '<S39>'  : 'Simulator/Airframe/Dynamics/Euler /pqr to Euler/Subsystem/Saturation Dynamic'
 * '<S40>'  : 'Simulator/Airframe/Dynamics/Euler /pqr to Euler/Subsystem/Edge Detector/Model'
 * '<S41>'  : 'Simulator/Airframe/Dynamics/Euler /pqr to Euler/Subsystem/Edge Detector/Model/Internal dirac generator'
 * '<S42>'  : 'Simulator/Airframe/Dynamics/Euler /pqr to Euler/Subsystem/Edge Detector/Model/NEGATIVE Edge'
 * '<S43>'  : 'Simulator/Airframe/Dynamics/Euler /pqr to Euler/Subsystem/Edge Detector/Model/POSITIVE Edge'
 * '<S44>'  : 'Simulator/Airframe/Dynamics/Euler /pqr to Euler/Subsystem/Edge Detector/Model/Internal dirac generator/Triggered Subsystem'
 * '<S45>'  : 'Simulator/Airframe/Dynamics/Rot accel to rot rate/Subsystem'
 * '<S46>'  : 'Simulator/Airframe/Dynamics/Rot accel to rot rate/Subsystem/Edge Detector'
 * '<S47>'  : 'Simulator/Airframe/Dynamics/Rot accel to rot rate/Subsystem/Saturation Dynamic'
 * '<S48>'  : 'Simulator/Airframe/Dynamics/Rot accel to rot rate/Subsystem/Edge Detector/Model'
 * '<S49>'  : 'Simulator/Airframe/Dynamics/Rot accel to rot rate/Subsystem/Edge Detector/Model/Internal dirac generator'
 * '<S50>'  : 'Simulator/Airframe/Dynamics/Rot accel to rot rate/Subsystem/Edge Detector/Model/NEGATIVE Edge'
 * '<S51>'  : 'Simulator/Airframe/Dynamics/Rot accel to rot rate/Subsystem/Edge Detector/Model/POSITIVE Edge'
 * '<S52>'  : 'Simulator/Airframe/Dynamics/Rot accel to rot rate/Subsystem/Edge Detector/Model/Internal dirac generator/Triggered Subsystem'
 * '<S53>'  : 'Simulator/Airframe/Dynamics/State reset logic/Default'
 * '<S54>'  : 'Simulator/Airframe/Dynamics/State reset logic/DoF'
 * '<S55>'  : 'Simulator/Airframe/Dynamics/State reset logic/Ground'
 * '<S56>'  : 'Simulator/Airframe/Dynamics/State reset logic/Platform'
 * '<S57>'  : 'Simulator/Airframe/Dynamics/State reset logic/Ground/S-R Flip-Flop'
 * '<S58>'  : 'Simulator/Airframe/Dynamics/Vbxw/Subsystem'
 * '<S59>'  : 'Simulator/Airframe/Dynamics/Vbxw/Subsystem1'
 * '<S60>'  : 'Simulator/Airframe/Dynamics/Ve to Xe/Subsystem'
 * '<S61>'  : 'Simulator/Airframe/Dynamics/Ve to Xe/Subsystem/Edge Detector'
 * '<S62>'  : 'Simulator/Airframe/Dynamics/Ve to Xe/Subsystem/Saturation Dynamic'
 * '<S63>'  : 'Simulator/Airframe/Dynamics/Ve to Xe/Subsystem/Edge Detector/Model'
 * '<S64>'  : 'Simulator/Airframe/Dynamics/Ve to Xe/Subsystem/Edge Detector/Model/Internal dirac generator'
 * '<S65>'  : 'Simulator/Airframe/Dynamics/Ve to Xe/Subsystem/Edge Detector/Model/NEGATIVE Edge'
 * '<S66>'  : 'Simulator/Airframe/Dynamics/Ve to Xe/Subsystem/Edge Detector/Model/POSITIVE Edge'
 * '<S67>'  : 'Simulator/Airframe/Dynamics/Ve to Xe/Subsystem/Edge Detector/Model/Internal dirac generator/Triggered Subsystem'
 * '<S68>'  : 'Simulator/Airframe/F&M/Actuators'
 * '<S69>'  : 'Simulator/Airframe/F&M/Aero parameters'
 * '<S70>'  : 'Simulator/Airframe/F&M/Aerodynamics'
 * '<S71>'  : 'Simulator/Airframe/F&M/Forces and Moments'
 * '<S72>'  : 'Simulator/Airframe/F&M/Gravity '
 * '<S73>'  : 'Simulator/Airframe/F&M/Actuators/motor filter'
 * '<S74>'  : 'Simulator/Airframe/F&M/Actuators/servo filter'
 * '<S75>'  : 'Simulator/Airframe/F&M/Actuators/motor filter/Integrator (Discrete or Continuous)'
 * '<S76>'  : 'Simulator/Airframe/F&M/Actuators/motor filter/Integrator (Discrete or Continuous)1'
 * '<S77>'  : 'Simulator/Airframe/F&M/Actuators/motor filter/Integrator (Discrete or Continuous)/Discrete'
 * '<S78>'  : 'Simulator/Airframe/F&M/Actuators/motor filter/Integrator (Discrete or Continuous)1/Discrete'
 * '<S79>'  : 'Simulator/Airframe/F&M/Actuators/servo filter/Integrator (Discrete or Continuous)'
 * '<S80>'  : 'Simulator/Airframe/F&M/Actuators/servo filter/Integrator (Discrete or Continuous)1'
 * '<S81>'  : 'Simulator/Airframe/F&M/Actuators/servo filter/Integrator (Discrete or Continuous)/Discrete'
 * '<S82>'  : 'Simulator/Airframe/F&M/Actuators/servo filter/Integrator (Discrete or Continuous)1/Discrete'
 * '<S83>'  : 'Simulator/Airframe/F&M/Aero parameters/Dynamic Pressure'
 * '<S84>'  : 'Simulator/Airframe/F&M/Aero parameters/Vanes'
 * '<S85>'  : 'Simulator/Airframe/F&M/Aero parameters/Dynamic Pressure/dot'
 * '<S86>'  : 'Simulator/Airframe/F&M/Aero parameters/Vanes/Subsystem'
 * '<S87>'  : 'Simulator/Airframe/F&M/Aero parameters/Vanes/Subsystem1'
 * '<S88>'  : 'Simulator/Airframe/F&M/Aero parameters/Vanes/dot'
 * '<S89>'  : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients'
 * '<S90>'  : 'Simulator/Airframe/F&M/Aerodynamics/Aero Coefficients'
 * '<S91>'  : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients'
 * '<S92>'  : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup'
 * '<S93>'  : 'Simulator/Airframe/F&M/Aerodynamics/Stab to Body'
 * '<S94>'  : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces'
 * '<S95>'  : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers'
 * '<S96>'  : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Aileron'
 * '<S97>'  : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Elevator'
 * '<S98>'  : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Flaps'
 * '<S99>'  : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Outboard ailerons'
 * '<S100>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Rudder'
 * '<S101>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Aileron/Subsystem Reference'
 * '<S102>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Aileron/Subsystem Reference/CL'
 * '<S103>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Aileron/Subsystem Reference/CX'
 * '<S104>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Aileron/Subsystem Reference/CY'
 * '<S105>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Aileron/Subsystem Reference/Cl'
 * '<S106>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Aileron/Subsystem Reference/Cm'
 * '<S107>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Aileron/Subsystem Reference/Cn'
 * '<S108>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Elevator/Subsystem Reference'
 * '<S109>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Elevator/Subsystem Reference/CL'
 * '<S110>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Elevator/Subsystem Reference/CX'
 * '<S111>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Elevator/Subsystem Reference/CY'
 * '<S112>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Elevator/Subsystem Reference/Cl'
 * '<S113>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Elevator/Subsystem Reference/Cm'
 * '<S114>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Elevator/Subsystem Reference/Cn'
 * '<S115>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Flaps/Subsystem Reference'
 * '<S116>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Flaps/Subsystem Reference/CL'
 * '<S117>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Flaps/Subsystem Reference/CX'
 * '<S118>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Flaps/Subsystem Reference/CY'
 * '<S119>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Flaps/Subsystem Reference/Cl'
 * '<S120>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Flaps/Subsystem Reference/Cm'
 * '<S121>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Flaps/Subsystem Reference/Cn'
 * '<S122>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Outboard ailerons/Subsystem Reference'
 * '<S123>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Outboard ailerons/Subsystem Reference/CL'
 * '<S124>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Outboard ailerons/Subsystem Reference/CX'
 * '<S125>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Outboard ailerons/Subsystem Reference/CY'
 * '<S126>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Outboard ailerons/Subsystem Reference/Cl'
 * '<S127>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Outboard ailerons/Subsystem Reference/Cm'
 * '<S128>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Outboard ailerons/Subsystem Reference/Cn'
 * '<S129>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Rudder/Subsystem Reference'
 * '<S130>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Rudder/Subsystem Reference/CL'
 * '<S131>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Rudder/Subsystem Reference/CX'
 * '<S132>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Rudder/Subsystem Reference/CY'
 * '<S133>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Rudder/Subsystem Reference/Cl'
 * '<S134>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Rudder/Subsystem Reference/Cm'
 * '<S135>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Control surfaces/Rudder/Subsystem Reference/Cn'
 * '<S136>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Combined thrust'
 * '<S137>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Thrust differencial'
 * '<S138>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Combined thrust/Subsystem Reference'
 * '<S139>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Combined thrust/Subsystem Reference/CL'
 * '<S140>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Combined thrust/Subsystem Reference/CX'
 * '<S141>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Combined thrust/Subsystem Reference/CY'
 * '<S142>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Combined thrust/Subsystem Reference/Cl'
 * '<S143>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Combined thrust/Subsystem Reference/Cm'
 * '<S144>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Combined thrust/Subsystem Reference/Cn'
 * '<S145>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Thrust differencial/Subsystem Reference'
 * '<S146>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Thrust differencial/Subsystem Reference/CL'
 * '<S147>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Thrust differencial/Subsystem Reference/CX'
 * '<S148>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Thrust differencial/Subsystem Reference/CY'
 * '<S149>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Thrust differencial/Subsystem Reference/Cl'
 * '<S150>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Thrust differencial/Subsystem Reference/Cm'
 * '<S151>' : 'Simulator/Airframe/F&M/Aerodynamics/Actuator Coefficients/Propellers/Thrust differencial/Subsystem Reference/Cn'
 * '<S152>' : 'Simulator/Airframe/F&M/Aerodynamics/Aero Coefficients/Subsystem Reference'
 * '<S153>' : 'Simulator/Airframe/F&M/Aerodynamics/Aero Coefficients/Subsystem Reference/CL'
 * '<S154>' : 'Simulator/Airframe/F&M/Aerodynamics/Aero Coefficients/Subsystem Reference/CX'
 * '<S155>' : 'Simulator/Airframe/F&M/Aerodynamics/Aero Coefficients/Subsystem Reference/CY'
 * '<S156>' : 'Simulator/Airframe/F&M/Aerodynamics/Aero Coefficients/Subsystem Reference/Cl'
 * '<S157>' : 'Simulator/Airframe/F&M/Aerodynamics/Aero Coefficients/Subsystem Reference/Cm'
 * '<S158>' : 'Simulator/Airframe/F&M/Aerodynamics/Aero Coefficients/Subsystem Reference/Cn'
 * '<S159>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/p Coefficients'
 * '<S160>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/q Coefficients'
 * '<S161>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/r Coefficients'
 * '<S162>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/p Coefficients/Subsystem Reference'
 * '<S163>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/p Coefficients/Subsystem Reference/CL'
 * '<S164>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/p Coefficients/Subsystem Reference/CX'
 * '<S165>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/p Coefficients/Subsystem Reference/CY'
 * '<S166>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/p Coefficients/Subsystem Reference/Cl'
 * '<S167>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/p Coefficients/Subsystem Reference/Cm'
 * '<S168>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/p Coefficients/Subsystem Reference/Cn'
 * '<S169>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/q Coefficients/Subsystem Reference'
 * '<S170>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/q Coefficients/Subsystem Reference/CL'
 * '<S171>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/q Coefficients/Subsystem Reference/CX'
 * '<S172>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/q Coefficients/Subsystem Reference/CY'
 * '<S173>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/q Coefficients/Subsystem Reference/Cl'
 * '<S174>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/q Coefficients/Subsystem Reference/Cm'
 * '<S175>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/q Coefficients/Subsystem Reference/Cn'
 * '<S176>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/r Coefficients/Subsystem Reference'
 * '<S177>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/r Coefficients/Subsystem Reference/CL'
 * '<S178>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/r Coefficients/Subsystem Reference/CX'
 * '<S179>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/r Coefficients/Subsystem Reference/CY'
 * '<S180>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/r Coefficients/Subsystem Reference/Cl'
 * '<S181>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/r Coefficients/Subsystem Reference/Cm'
 * '<S182>' : 'Simulator/Airframe/F&M/Aerodynamics/Body Rate Damping Coefficients/r Coefficients/Subsystem Reference/Cn'
 * '<S183>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab'
 * '<S184>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Degrees to Radians'
 * '<S185>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Radians to Degrees'
 * '<S186>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Radians to Degrees1'
 * '<S187>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/A11'
 * '<S188>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/A12'
 * '<S189>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/A13'
 * '<S190>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/A21'
 * '<S191>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/A22'
 * '<S192>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/A23'
 * '<S193>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/A31'
 * '<S194>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/A32'
 * '<S195>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/A33'
 * '<S196>' : 'Simulator/Airframe/F&M/Aerodynamics/PreLookup/Body to stab/Create Transformation Matrix'
 * '<S197>' : 'Simulator/Airframe/F&M/Forces and Moments/Denormalize & Transform to CG'
 * '<S198>' : 'Simulator/Airframe/F&M/Forces and Moments/Denormalize & Transform to CG/3x3 Cross Product'
 * '<S199>' : 'Simulator/Airframe/F&M/Forces and Moments/Denormalize & Transform to CG/CG-CP Transformation'
 * '<S200>' : 'Simulator/Airframe/F&M/Forces and Moments/Denormalize & Transform to CG/Force Transformation'
 * '<S201>' : 'Simulator/Airframe/F&M/Forces and Moments/Denormalize & Transform to CG/Moment Transformation'
 * '<S202>' : 'Simulator/Airframe/F&M/Forces and Moments/Denormalize & Transform to CG/3x3 Cross Product/Subsystem'
 * '<S203>' : 'Simulator/Airframe/F&M/Forces and Moments/Denormalize & Transform to CG/3x3 Cross Product/Subsystem1'
 * '<S204>' : 'Simulator/Control/Controller selection'
 * '<S205>' : 'Simulator/Control/Controller selection/Ardupilot'
 * '<S206>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions'
 * '<S207>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace'
 * '<S208>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Positive Trace'
 * '<S209>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM'
 * '<S210>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/trace(DCM)'
 * '<S211>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)'
 * '<S212>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)'
 * '<S213>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)'
 * '<S214>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/diag(DCM)'
 * '<S215>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S216>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S217>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S218>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/if s~=0; s=0.5//s'
 * '<S219>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(1,1)/u(1) -(u(5)+u(9)) +1'
 * '<S220>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S221>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S222>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/cos(theta)sin(psi) + (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S223>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/if s~=0; s=0.5//s'
 * '<S224>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(2,2)/u(5) -(u(1)+u(9)) +1'
 * '<S225>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) -sin(theta)'
 * '<S226>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(phi) + (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S227>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S228>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/if s~=0; s=0.5//s'
 * '<S229>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Negative Trace/Maximum Value at DCM(3,3)/u(9) -(u(1)+u(5)) +1'
 * '<S230>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi) +sin(theta)'
 * '<S231>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(phi) - (cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi))'
 * '<S232>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Positive Trace/cos(theta)sin(psi) - (sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi))'
 * '<S233>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error'
 * '<S234>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal'
 * '<S235>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper'
 * '<S236>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal'
 * '<S237>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper'
 * '<S238>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Error'
 * '<S239>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/Else If Not Orthogonal/Warning'
 * '<S240>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Error'
 * '<S241>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/If Not Proper/Warning'
 * '<S242>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotOrthogonal/transpose*dcm ~= eye(3)'
 * '<S243>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/Determinant of 3x3 Matrix'
 * '<S244>' : 'Simulator/Control/Controller selection/Ardupilot/Direction Cosine Matrix  to Quaternions/Validate DCM/If Warning//Error/isNotProper/determinant does not equal 1'
 * '<S245>' : 'Simulator/Environment/Gravity & Density fields'
 * '<S246>' : 'Simulator/Environment/Wind model'
 * '<S247>' : 'Simulator/Environment/Gravity & Density fields/COESA Atmosphere Model'
 * '<S248>' : 'Simulator/Environment/Gravity & Density fields/COESA Atmosphere Model/Density Conversion'
 * '<S249>' : 'Simulator/Environment/Gravity & Density fields/COESA Atmosphere Model/Length Conversion'
 * '<S250>' : 'Simulator/Environment/Gravity & Density fields/COESA Atmosphere Model/Pressure Conversion'
 * '<S251>' : 'Simulator/Environment/Gravity & Density fields/COESA Atmosphere Model/Temperature Conversion'
 * '<S252>' : 'Simulator/Environment/Gravity & Density fields/COESA Atmosphere Model/Velocity Conversion'
 * '<S253>' : 'Simulator/Environment/Wind model/Discrete Wind Gust Model'
 * '<S254>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))'
 * '<S255>' : 'Simulator/Environment/Wind model/Wind Shear Model'
 * '<S256>' : 'Simulator/Environment/Wind model/Discrete Wind Gust Model/Distance into gust (x)'
 * '<S257>' : 'Simulator/Environment/Wind model/Discrete Wind Gust Model/Distance into gust (y)'
 * '<S258>' : 'Simulator/Environment/Wind model/Discrete Wind Gust Model/Distance into gust (z)'
 * '<S259>' : 'Simulator/Environment/Wind model/Discrete Wind Gust Model/Velocity Conversion'
 * '<S260>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Angle Conversion'
 * '<S261>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Filters on angular rates'
 * '<S262>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Filters on velocities'
 * '<S263>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Length Conversion'
 * '<S264>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Length Conversion1'
 * '<S265>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/RMS turbulence  intensities'
 * '<S266>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select angular rates'
 * '<S267>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select velocities'
 * '<S268>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Turbulence scale lengths'
 * '<S269>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Velocity Conversion'
 * '<S270>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Velocity Conversion2'
 * '<S271>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/White Noise'
 * '<S272>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Filters on angular rates/Hpgw'
 * '<S273>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Filters on angular rates/Hqgw'
 * '<S274>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Filters on angular rates/Hrgw'
 * '<S275>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Filters on velocities/Hugw(s)'
 * '<S276>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Filters on velocities/Hvgw(s)'
 * '<S277>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Filters on velocities/Hwgw(s)'
 * '<S278>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/RMS turbulence  intensities/High Altitude Intensity'
 * '<S279>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/RMS turbulence  intensities/Low Altitude Intensity'
 * '<S280>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select angular rates/Interpolate  rates'
 * '<S281>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select angular rates/Low altitude  rates'
 * '<S282>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select angular rates/Medium//High  altitude rates'
 * '<S283>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select angular rates/Merge Subsystems'
 * '<S284>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select angular rates/Interpolate  rates/wind to body transformation'
 * '<S285>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
 * '<S286>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select angular rates/Low altitude  rates/wind to body transformation'
 * '<S287>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
 * '<S288>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select velocities/Interpolate  velocities'
 * '<S289>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select velocities/Low altitude  velocities'
 * '<S290>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select velocities/Medium//High  altitude velocities'
 * '<S291>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select velocities/Merge Subsystems'
 * '<S292>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select velocities/Interpolate  velocities/wind to body transformation'
 * '<S293>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
 * '<S294>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select velocities/Low altitude  velocities/wind to body transformation'
 * '<S295>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
 * '<S296>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Turbulence scale lengths/Low altitude scale length'
 * '<S297>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Turbulence scale lengths/Medium//High altitude scale length'
 * '<S298>' : 'Simulator/Environment/Wind model/Dryden Wind Turbulence Model  (Continuous (+q -r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
 * '<S299>' : 'Simulator/Environment/Wind model/Wind Shear Model/Angle Conversion'
 * '<S300>' : 'Simulator/Environment/Wind model/Wind Shear Model/Length Conversion'
 * '<S301>' : 'Simulator/Graph/Angle Conversion2'
 * '<S302>' : 'Simulator/Graph/Angle Conversion3'
 * '<S303>' : 'Simulator/Graph/Angle Conversion4'
 * '<S304>' : 'Simulator/Graph/Incidence, Sideslip, & Airspeed'
 * '<S305>' : 'Simulator/Graph/Length Conversion'
 * '<S306>' : 'Simulator/Graph/Incidence, Sideslip, & Airspeed/Subsystem'
 * '<S307>' : 'Simulator/Graph/Incidence, Sideslip, & Airspeed/Subsystem1'
 * '<S308>' : 'Simulator/Graph/Incidence, Sideslip, & Airspeed/dot'
 * '<S309>' : 'Simulator/Output/None'
 */
#endif                                 /* RTW_HEADER_Simulator_h_ */

#pragma GCC diagnostic pop
