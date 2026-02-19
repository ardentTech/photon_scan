#include "utils.h"

// Re-maps `a` from range `in_min`-`in_max` to range `out_min`-`out_max`.
uint16_t map(uint16_t a, const uint16_t in_min, const uint16_t in_max, const uint16_t out_min, const uint16_t out_max) {
    return (a - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
