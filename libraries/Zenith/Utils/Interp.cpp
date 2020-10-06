#include "Interp.h"

template <typename T>
Interp<T>::Interp(const std::vector<const std::vector<T>> &lookups, oorBehavior_e oorBehavior = CLIP) 
    : lookups(lookups), oorBehavior(oorBehavior) {
        assert(lookups.size() > 0 && lookups.size() < 32);
        lookupProd = 1;
        for (auto lookup : lookups) {
            // Lookup must be large enough for interpolation
            assert(lookup.size() > 1);
            // Assert that the lookup is monotically increasing
            for (size_t k = 0; k < lookup.size() - 1; k++)
                assert(lookup[k] < lookup[k + 1]);
            // Increment table size
            lookupProd *= lookup.size();
        }
    };

template <typename T>
std::vector<T> Interp<T>::clamp(const std::vector<T> &vector) const {
    std::vector<T> rtn;
    for (size_t k = 0; k < lookups.size(); k++) {
        T min = lookups[k][0];
        T max = lookups[k][lookups.size() - 1];
        rtn.push_back(MAX(min, MIN(max, vector[k])));
    }
    return rtn;
};

template <typename T>
T Interp<T>::get(const T *table, const std::vector<T> &values, size_t tableIdx = 0) const {
    assert(values.size() == lookups.size());
    std::vector<lookupIdx_t> indices;
    std::vector<size_t> indList;
    for (size_t k = 0; k < lookups.size(); k++) {
        auto val = values[k];
        indices.push_back(findIndex(k, val));
        indList.push_back(0);
    }
    // Now loop through all possible indices n-uplets
    T value = 0.0;
    for (size_t k = 0; k < 1 << lookups.size(); k++) {
        T frac = 1;
        for (size_t l = 0; l < lookups.size(); l++) {
            size_t on = (k >> l) & 1;
            indList[l] = indices[l].idx + on;
            frac *= on ? indices[l].frac : 1 - indices[l].frac;
        }
        // Now get value at indList
        value += frac * getValue(table, indList, tableIdx);
    }
    return value;
};

template <typename T>
Interp<T>::lookupIdx_t Interp<T>::findIndex(size_t lookupIdx, T value) const {
    auto lookup = lookups[lookupIdx];
    size_t k;
    for (k = 0; (k < lookup.size() - 2) && (value > lookup[k + 1]); k++);
    T frac = (value - lookup[k]) / (lookup[k + 1] - lookup[k]);
    if (oorBehavior == CLIP)
        frac = MAX(0.0, MIN(1.0, frac));
    return (lookupIdx_t) {
        .idx = k,
        .frac = frac
    };
};

template <typename T>
T Interp<T>::getValue(const T *table, const std::vector<size_t> &indices, size_t tableIdx) const {
    assert(indices.size() > 0);
    size_t idx = tableIdx;
    for (size_t l = 0; l < lookups.size(); l++) {
        idx = idx * lookups[l].size() + indices[l];
    };
    return table[idx];
};
