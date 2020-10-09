// #include "Interp.h"

template <typename T>
Interp<T>::Interp(const std::vector<const std::vector<T>> &lookups, oorBehavior_e _oor_behavior) 
    : lookups(lookups) {
        assert(lookups.size() > 0 && lookups.size() < 32);
        lookup_prod = 1;
        for (auto lookup : lookups) {
            // Lookup must be large enough for interpolation
            assert(lookup.size() > 1);
            // Assert that the lookup is monotically increasing
            for (size_t k = 0; k < lookup.size() - 1; k++)
                assert(lookup[k] < lookup[k + 1]);
            // Increment table size
            lookup_prod *= lookup.size();
        }
        oor_behavior = _oor_behavior;
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
T Interp<T>::get(const T *table, size_t table_length, const std::vector<T> &values, size_t vec_length,  size_t vec_idx) const {
    // printf("table length %lu lookup_prod %lu vec_length %lu values.size %lu lookups.size %lu vec_idx %lu vec_length %lu\n", table_length, lookup_prod, vec_length, values.size(), lookups.size(), vec_idx, vec_length);
    assert(table_length == lookup_prod * vec_length && values.size() == lookups.size() && vec_idx < vec_length);
    std::vector<lookupIdx_t> indices;
    std::vector<size_t> indList;
    for (size_t k = 0; k < lookups.size(); k++) {
        auto val = values[k];
        indices.push_back(find_index(k, val));
        indList.push_back(0);
    }
    // Now loop through all possible indices n-uplets
    // Perform a multilinear interpolation
    T value = 0.0;
    for (size_t k = 0; k < 1 << lookups.size(); k++) {
        T frac = 1;
        for (size_t l = 0; l < lookups.size(); l++) {
            size_t on = (k >> l) & 1;
            indList[l] = indices[l].idx + on;
            frac *= on ? indices[l].frac : 1 - indices[l].frac;
        }
        // Now get value at indList
        value += frac * get_value(table, table_length, indList, vec_length, vec_idx);
    }
    return value;
};

template <typename T>
std::vector<T> Interp<T>::get_vec(const T *table, size_t table_length, const std::vector<T> &values) const {
    size_t length = table_length / lookup_prod;
    std::vector<T> vec(length);
    for (size_t k = 0; k < length; k++)
        vec[k] = get(table, table_length, values, length, k);
    return vec;
};

template <typename T>
std::vector<T> Interp<T>::get_vec(const std::vector<T> &table, const std::vector<T> &values) const {
    return get_vec(&table[0], table.size(), values);
}

template <typename T>
typename Interp<T>::lookupIdx_t Interp<T>::find_index(size_t lookup_idx, T value) const {
    auto lookup = lookups[lookup_idx];
    size_t k;
    for (k = 0; (k < lookup.size() - 2) && (value > lookup[k + 1]); k++);
    T frac = (value - lookup[k]) / (lookup[k + 1] - lookup[k]);
    if (oor_behavior == CLIP)
        frac = MAX(0.0, MIN(1.0, frac));
    return (lookupIdx_t) {
        .idx = k,
        .frac = frac
    };
};

template <typename T>
T Interp<T>::get_value(const T *table, size_t table_length, const std::vector<size_t> &indices, size_t vec_length,  size_t vec_idx) const {
    // printf("table length %lu lookup_prod %lu vec_length %lu indices.size %lu lookups.size %lu vec_idx %lu vec_length %lu\n", table_length, lookup_prod, vec_length, indices.size(), lookups.size(), vec_idx, vec_length);
    assert(table_length == lookup_prod * vec_length && indices.size() == lookups.size() && vec_idx < vec_length);
    size_t idx = vec_idx;
    for (size_t l = 0; l < lookups.size(); l++)
        idx = idx * lookups[l].size() + indices[l];
    return table[idx];
};
