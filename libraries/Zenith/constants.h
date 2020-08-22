// Models definitions
#define MODEL_XUAV_ASW 0
#define MODEL_XUAV_AVL 1
#define MODEL_Z1F_ASW 2
#define MODEL_Z1F_AVL 3

#define ZENITH_MODEL MODEL_XUAV_AVL

// Assign aero data
#if ZENITH_MODEL == MODEL_XUAV_AVL
#include "Simulator/data/xuav_config.h"
#include "Simulator/data/xuav_aeroData.h"
#include "Controller/data/xuav_controllerData.h"
#define ModelConfig xuav_config
#define ModelAeroData xuav_aeroData
#define ControllerData xuav_controllerData
#else
#error Compile with --model flag
#endif

// Common macros
#define DIM(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))