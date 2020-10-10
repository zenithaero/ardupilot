#pragma once

#include <vector>

template <typename T>
class Interp {

struct lookupIdx_t {
    const size_t idx;
    const T frac;
};

typedef enum {
    CLIP,
    INTERPOLATE
} oorBehavior_e;

public:
    Interp(const std::vector<std::vector<T>> &_lookups, oorBehavior_e _oor_behavior = CLIP);

    std::vector<T> clamp(const std::vector<T> &vector) const;
    
    T get(const T *table, size_t table_length, const std::vector<T> &values, size_t vec_length = 1,  size_t vec_idx = 0) const;

    std::vector<T> get_vec(const T *table, size_t table_length, const std::vector<T> &values) const;

    std::vector<T> get_vec(const std::vector<T> &table, const std::vector<T> &values) const;

private:
    const std::vector<std::vector<T>> lookups;
    oorBehavior_e oor_behavior;
    size_t lookup_prod;

    lookupIdx_t find_index(size_t lookupIdx, T value) const;

    T get_value(const T *table, size_t table_length, const std::vector<size_t> &indices, size_t vec_length = 1,  size_t vec_idx = 0) const;
};

#include "Interp.cpp" // Required for template classes