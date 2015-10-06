/*
 * Wave functions on the XZ plane.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include "lib/prng.h"
#include "lib/effect.h"
#include "lib/texture.h"


class PlanarWaves : public Effect {
public:
    PlanarWaves();

    void reset();

    virtual void beginFrame(const FrameInfo &f);
    virtual void shader(Vec3& rgb, const PixelInfo& p) const;

    struct Wave {
        float targetAmplitude;
        float targetFrequency;
        float targetAngle;
        float targetPhase;

        float amplitude;
        float frequency;
        float angle;
        float phase;

        Wave();

        float eval(Vec2 p) const;
        void step(const FrameInfo &f);
        void set();
    };

    // Y axis on palette
    float targetColorParam;
    float colorParam;

    Vec2 speed;
    Vec2 speedTarget;
    Vec2 center;

    std::vector<Wave> waves;

private:
    static constexpr float stepSize = 1.0 / 500;
    static constexpr float easingRate = 0.01;

    Texture palette;
    float timeDeltaRemainder;

    void runStep(const FrameInfo &f);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline PlanarWaves::PlanarWaves()
    : palette("data/wave-palette.png"),
      timeDeltaRemainder(0)
{}

inline void PlanarWaves::reset()
{
    colorParam = targetColorParam = 0;
    speed = Vec2(0,0);
    center = Vec2(0,0);
    waves.clear();
}

inline void PlanarWaves::beginFrame(const FrameInfo &f)
{
    float t = f.timeDelta + timeDeltaRemainder;
    int steps = t / stepSize;
    timeDeltaRemainder = t - steps * stepSize;

    while (steps > 0) {
        runStep(f);
        steps--;
    }
}

inline void PlanarWaves::shader(Vec3& rgb, const PixelInfo& p) const
{
    Vec2 pt = Vec2(p.point[0], p.point[2]) - center;
    float a = 0.5;

    for (unsigned i = 0; i < waves.size(); i++) {
        a += waves[i].eval(pt);
    }

    rgb = palette.sample(a, colorParam);
}

inline void PlanarWaves::Wave::set()
{
    targetAmplitude = amplitude;
    targetPhase = phase;
    targetFrequency = frequency;
    targetAngle = angle;
}

inline float PlanarWaves::Wave::eval(Vec2 p) const
{
    // Rotate in XZ plane
    float x = p[0] * cos(angle) - p[1] * sin(angle);

    // Stationary point at the origin, for predictable transition in
    return amplitude * sin(x * frequency + phase);
}

inline void PlanarWaves::runStep(const FrameInfo &f)
{
    colorParam += (targetColorParam - colorParam) * easingRate;
    speed += (speedTarget - speed) * easingRate;
    center += speed;

    for (unsigned i = 0; i < waves.size(); i++) {
        waves[i].step(f);
    }
}

inline void PlanarWaves::Wave::step(const FrameInfo &f)
{
    amplitude += (targetAmplitude - amplitude) * easingRate;
    frequency += (targetFrequency - frequency) * easingRate;
    angle += (targetAngle - angle) * easingRate;
    phase += (targetPhase - phase) * easingRate;
}

inline PlanarWaves::Wave::Wave()
{
    targetAmplitude = amplitude = 0;
    targetFrequency = frequency = 0;
    targetAngle = angle = 0;
    targetPhase = phase = 0;
}
