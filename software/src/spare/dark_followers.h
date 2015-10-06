/*
 * Specks of darkness trace a path through space.
 * Space illustrates time.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <vector>
#include "lib/particle.h"
#include "lib/prng.h"
#include "lib/noise.h"
#include "lib/texture.h"


class DarkFollowers : public ParticleEffect
{
public:
    DarkFollowers();
    void reseed(unsigned seed);

    void snap(Vec3 location, float rate);
    void push(Vec3 location, float radius, Vec3 displacement);

private:
    static constexpr unsigned numParticles = 400;
    static constexpr float seedRadius = 4.0;
    static constexpr float radius = 0.2;
    static constexpr float intensity = 6.0;
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline DarkFollowers::DarkFollowers()
{
    reseed(42);
}

inline void DarkFollowers::reseed(unsigned seed)
{
    appearance.resize(numParticles);

    PRNG prng;
    prng.seed(seed);

    for (unsigned i = 0; i < appearance.size(); i++) {
        Vec2 p = prng.ringVector(1e-4, seedRadius);
        appearance[i].point = Vec3(p[0], 0, p[1]);
        appearance[i].intensity = intensity;
        appearance[i].radius = radius;
        appearance[i].color = Vec3(-1, -1, -1);
    }
}

inline void DarkFollowers::snap(Vec3 location, float rate)
{
    for (unsigned i = 0; i < appearance.size(); i++) {
        Vec3& p = appearance[i].point;
        p += (location - p) * rate;
    }
}

inline void DarkFollowers::push(Vec3 location, float radius, Vec3 displacement)
{
    std::vector<std::pair<size_t, Real> > hits;
    index.radiusSearch(hits, location, radius);

    for (unsigned i = 0; i < numHits; i++) {
        ParticleAppearance &hit = appearance[hits[i].first];
        float q2 = hits[i].second / sq(radius);
        if (q2 < 1.0f) {
            hit.point += displacement * kernel2(q2);
        }
    }
}
