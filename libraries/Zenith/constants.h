#pragma once

#include <vector>

// Models definitions
#define MODEL_XUAV_ASW 0
#define MODEL_Z1F_ASW 1

#define ZENITH_MODEL MODEL_XUAV_ASW

// Assign aero data
#if ZENITH_MODEL == MODEL_XUAV_ASW
#include "Simulator/data/xuav_config.h"
#include "Simulator/data/xuav_aeroData.h"
#include "Controller/data/xuav_controllerData.h"
#define ModelConfig xuav_config
#define ModelAeroData xuav_aeroData
#define ControllerData xuav_controllerData
#elif ZENITH_MODEL == MODEL_Z1F_ASW
#include "Simulator/data/Z1f_config.h"
#include "Simulator/data/Z1f_aeroData.h"
#include "Controller/data/Z1f_controllerData.h"
#define ModelConfig Z1f_config
#define ModelAeroData Z1f_aeroData
#define ControllerData Z1f_controllerData
#else
#error Undefined model
#endif

// Common macros
#define DIM(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#define CLAMP(value, min, max) MAX(min, MIN(max, value))

bool soft_assert(bool assertion, const char* format, ...) {
    va_list argptr;
    va_start(argptr, format);
    if (!assertion)
        vprintf(format, argptr);
    va_end(argptr);
    return !assertion;
}

void hard_assert(bool assertion, const char* format, ...) {
    va_list argptr;
    if (soft_assert(assertion, format, argptr))
        exit(-1);
}

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T>
std::vector<T> matmul(std::vector<std::vector<T>> mat, std::vector<T> vec) {
    std::vector<float> rtn(mat.size(), 0);
    if (vec.size() != mat[0].size()) {
        printf("Invalid vector size: %lu, expected %lu\n", vec.size(), mat[0].size());
        return rtn;
    }
    for (size_t i = 0; i < mat.size(); i++)
        for (size_t j = 0; j < mat[0].size(); j++)
            rtn[i] += mat[i][j] * vec[j];
    return rtn;
}

template <typename T, size_t m, size_t n>
void build_mat(const T (&data)[m][n], std::vector<std::vector<T>> &mat) {
    for (size_t i = 0; i < m; i++) {
		std::vector<T> row(data[i], data[i] + n);
		mat.push_back(row);
	}
}

template <typename T>
void reshape(std::vector<T> &src, std::vector<std::vector<T>> &dest) {
    if (dest.size() == 0 ||  src.size() != dest.size() * dest[0].size()) {
        printf("Invalid sizes\n");
        return;
    }
    for (size_t i = 0; i < dest.size(); i++)
        for (size_t j = 0; j < dest[0].size(); j++)
            dest[i][j] = src[i * dest[0].size() + j];
}

template <typename T>
void print_vec(std::vector<T> &vec, char *str) {
    printf("%s: [", str);
	for (size_t i = 0; i < vec.size(); i++)
			printf("%.2f, ", vec[i]);
    printf("]\n");
}

template <typename T>
void print_mat(std::vector<T> &mat, char *str) {
    printf("%s: \n", str);
    printf("[");
	for (size_t i = 0; i < mat.size(); i++) {
		for (size_t j = 0; j < mat[0].size(); j++)
			printf("%.2f, ", mat[i][j]);
		if (i == mat.size() - 1)
			printf("]");
		printf("\n");
	}
}