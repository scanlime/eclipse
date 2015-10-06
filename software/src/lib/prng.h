/*
 * A tiny, fast, and predictable public domain PRNG.
 *
 * A C++ adaptation of:
 *   http://burtleburtle.net/bob/rand/smallprng.html
 */

#pragma once
#include <stdint.h>
#include "svl/SVL.h"

class PRNG {
private:
    uint32_t rng0, rng1, rng2, rng3;

public:
    /**
     * Thoroughly but relatively slowly reinitialize the PRNG state
     * based on a provided 32-bit value. This runs the algorithm for
     * enough rounds to ensure good mixing.
     */
    void seed(uint32_t s);

    /**
     * This quickly but haphazardly mixes additional entropy into
     * the PRNG without fully re-seeding it.
     */
    void remix(uint32_t v);

    uint32_t uniform32();
    double uniform();
    double uniform(double a, double b);
    Vec2 circularVector();
    Vec2 ringVector(Real min, Real max);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/

inline void PRNG::seed(uint32_t s)
{
    rng0 = 0xf1ea5eed;
    rng1 = rng2 = rng3 = s;
    for (unsigned i = 0; i < 20; ++i)
        uniform32();
}

/**
 * This quickly but haphazardly mixes additional entropy into
 * the PRNG without fully re-seeding it.
 */
inline void PRNG::remix(uint32_t v)
{
    rng3 ^= v;
    rng0 ^= v;
}

inline uint32_t __attribute__((always_inline)) PRNG::uniform32()
{
    uint32_t rng4 = (rng0 - ((rng1 << 27) | (rng1 >> 5)));
    rng0 = rng1 ^ ((rng2 << 17) | (rng2 >> 15));
    rng1 = rng2 + rng3;
    rng2 = rng3 + rng4;
    rng3 = rng4 + rng0;
    return rng3;
}

inline double __attribute__((always_inline)) PRNG::uniform()
{
    return uniform32() * 2.3283064365386963e-10;
}

inline double __attribute__((always_inline)) PRNG::uniform(double a, double b)
{
    return a + uniform() * (b - a);
}

inline Vec2 PRNG::circularVector()
{
    return ringVector(0, 1);
}

inline Vec2 PRNG::ringVector(Real min, Real max)
{
    // Random vector with a length between 'min' and 'max'

    float extent = std::max(std::max(-min, min), std::max(-max, max));

    for (unsigned i = 0; i < 200; i++) {
        Vec2 v;
        v[0] = uniform(-extent, extent);
        v[1] = uniform(-extent, extent);
        Real sl = sqrlen(v);
        if (sl <= max * max && sl >= min * min) {
            return v;
        }
    }
    return Vec2(0, 0);
}
