#pragma once

#include <vector>
#include <iostream>
#include <iomanip>
#include <cstdint> 
#include <math.h> 

extern int here_count;
#define HERE(x) std::cout << "!!!HERE!!! " << x << " " << here_count++ << std::endl;

// Convert seconds to ticks, usually microseconds
#define SEC_TO_TICKS(v) (static_cast<int>(rint(v*1e6)))

using IntVec = std::vector<int32_t> ;

// Return -1 , 0  or 1 to indicate the sign of the number. 
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool same_sign(float a, float b);

// To convert class enums to ints. 
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
    return static_cast<typename std::underlying_type<E>::type>(e);
}


