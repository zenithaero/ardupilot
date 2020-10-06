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
    Interp(const std::vector<const std::vector<T>> &lookups, oorBehavior_e oorBehavior = CLIP);

    std::vector<T> clamp(const std::vector<T> &vector) const;
    
    T get(const T *table, const std::vector<T> &values, size_t tableIdx = 0) const;

private:
    const std::vector<const std::vector<T>> lookups;
    oorBehavior_e oorBehavior;
    size_t lookupProd;

    lookupIdx_t findIndex(size_t lookupIdx, T value) const;

    T getValue(const T *table, const std::vector<size_t> &indices, size_t tableIdx) const;
};