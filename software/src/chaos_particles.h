/*
 * Complex particle system.
 * Inspired by fractals,
 * leading to chaos.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <vector>
#include "lib/camera_flow.h"
#include "lib/particle.h"
#include "lib/prng.h"
#include "lib/texture.h"


class ChaosParticles : public ParticleEffect
{
public:
    ChaosParticles(const CameraFlowAnalyzer &flow, const rapidjson::Value &config);
    void reseed(Vec2 location, unsigned seed);

    bool isRunning();
    float getTotalIntensity();

    virtual void beginFrame(const FrameInfo &f);
    virtual void debug(const DebugInfo &di);

private:
    unsigned numParticles;
    unsigned numDarkParticles;
    unsigned maxAge;
    float generationScale;
    float speedMin;
    float speedMax;
    float spinMin;
    float spinMax;
    float relativeSize;
    float intensity;
    float intensityExp;
    float initialSpeedMin;
    float initialSpeedMax;
    float stepSize;
    float colorRate;
    float outsideMargin;
    float darkMultiplier;
    float flowScaleTarget;
    float flowScaleRampRate;
    float flowFilterRate;

    struct ParticleDynamics {
        Vec2 position;
        Vec2 velocity;
        bool escaped, dead;
        unsigned generation;
        unsigned age;
    };

    CameraFlowCapture flow;

    Texture palette;
    std::vector<ParticleDynamics> dynamics;
    float timeDeltaRemainder;
    float colorCycle;
    float totalIntensity;
    float flowScale;
    bool running;

    void runStep(const FrameInfo &f);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/

inline ChaosParticles::ChaosParticles(const CameraFlowAnalyzer &flow, const rapidjson::Value &config)
    : numParticles(config["numParticles"].GetUint()),
      numDarkParticles(config["numDarkParticles"].GetUint()),
      maxAge(config["maxAge"].GetUint()),
      generationScale(config["generationScale"].GetDouble()),
      speedMin(config["speedMin"].GetDouble()),
      speedMax(config["speedMax"].GetDouble()),
      spinMin(config["speedMin"].GetDouble()),
      spinMax(config["speedMax"].GetDouble()),
      relativeSize(config["relativeSize"].GetDouble()),
      intensity(config["intensity"].GetDouble()),
      intensityExp(config["intensityExp"].GetDouble()),
      initialSpeedMin(config["initialSpeedMin"].GetDouble()),
      initialSpeedMax(config["initialSpeedMax"].GetDouble()),
      stepSize(config["stepSize"].GetDouble()),
      colorRate(config["colorRate"].GetDouble()),
      outsideMargin(config["outsideMargin"].GetDouble()),
      darkMultiplier(config["darkMultiplier"].GetDouble()),
      flowScaleTarget(config["flowScaleTarget"].GetDouble()),
      flowScaleRampRate(config["flowScaleRampRate"].GetDouble()),
      flowFilterRate(config["flowFilterRate"].GetDouble()),
      flow(flow),
      palette(config["palette"].GetString()),
      timeDeltaRemainder(0),
      colorCycle(0)
{
    reseed(Vec2(0,0), 42);
}

inline bool ChaosParticles::isRunning()
{
    return running;
}

inline float ChaosParticles::getTotalIntensity()
{
    return totalIntensity;
}

inline void ChaosParticles::reseed(Vec2 location, unsigned seed)
{
    running = true;
    totalIntensity = nanf("");
    flowScale = 0;

    appearance.resize(numParticles);
    dynamics.resize(numParticles);

    PRNG prng;
    prng.seed(seed);

    flow.capture(1.0);
    flow.origin();

    colorCycle = prng.uniform(0, M_PI * 2);

    for (unsigned i = 0; i < dynamics.size(); i++) {
        dynamics[i].position = location;
        dynamics[i].velocity = prng.ringVector(initialSpeedMin, initialSpeedMax);
        dynamics[i].age = 0;
        dynamics[i].generation = 0;
        dynamics[i].dead = false;
        dynamics[i].escaped = false;
    }
}

inline void ChaosParticles::beginFrame(const FrameInfo &f)
{    
    if (running) {
        float t = f.timeDelta + timeDeltaRemainder;
        int steps = t / stepSize;
        timeDeltaRemainder = t - steps * stepSize;

        while (steps > 0) {
            runStep(f);
            steps--;
        }

        colorCycle = fmodf(colorCycle + f.timeDelta * colorRate, 2 * M_PI);
    }

    ParticleEffect::beginFrame(f);
}

inline void ChaosParticles::debug(const DebugInfo &di)
{
    fprintf(stderr, "\t[chaos-particles] running = %d\n", running);
    fprintf(stderr, "\t[chaos-particles] totalIntensity = %f\n", totalIntensity);
    fprintf(stderr, "\t[chaos-particles] flowScale = %f\n", flowScale);
    ParticleEffect::debug(di);
}

inline void ChaosParticles::runStep(const FrameInfo &f)
{
    PRNG prng;
    prng.seed(19);

    unsigned numLiveParticles = 0;
    float intensityAccumulator = 0;

    // Capture the impulse between the last step and this one
    flow.capture(flowFilterRate);
    flowScale = std::min(flowScaleTarget, flowScale + flowScaleRampRate * stepSize);

    // Update dynamics
    for (unsigned i = 0; i < dynamics.size(); i++) {
        ParticleDynamics pd = dynamics[i];
        ParticleAppearance pa = appearance[i];

        pd.age++;
        if (pd.age > maxAge && !pd.dead) {
            pd.dead = true;
            pa.intensity = 0;
            dynamics[i] = pd;
            appearance[i] = pa;
        }
        if (pd.dead) {
            continue;
        }

        // XZ plane
        // Horizontal flow -> position
        pa.point[0] = pd.position[0] - flow.model[0] * flowScale;
        pa.point[2] = pd.position[1];

        float ageF = pd.age / (float)maxAge;
        float c = (pd.generation + ageF) * generationScale;

        // Fade in/out
        float fade = pow(std::max(0.0f, sinf(ageF * M_PI)), intensityExp);
        float particleIntensity = intensity * fade;

        pa.radius = f.modelRadius * relativeSize * fade;
        numLiveParticles++;

        // Dark matter, to break up the monotony of lightness
        bool darkParticle = i < numDarkParticles;
        pa.intensity = darkParticle ? particleIntensity * darkMultiplier : particleIntensity;
        pa.color = darkParticle ? Vec3(1,1,1) : palette.sample(c, 0.5 + 0.5 * sinf(colorCycle));
        intensityAccumulator += darkParticle ? particleIntensity : 0;

        pd.position += pd.velocity;
        pd.escaped = f.distanceOutsideBoundingBox(pa.point) >
            outsideMargin * pa.radius;

        // Respawn escaped particles near a random other particle;
        // Appearance won't update until the next step.
        if (pd.escaped) {
            for (unsigned attempt = 0; attempt < 100; attempt++) {
                unsigned seed = prng.uniform(0, dynamics.size() - 0.0001);
                if (!dynamics[seed].escaped) {

                    // Fractal respawn at seed position
                    pd = dynamics[seed];
                    pd.generation++;
                    pd.age = 0;
                    Vec2 v = pd.velocity;

                    // Speed modulation
                    v *= prng.uniform(speedMin, speedMax);

                    // Direction modulation
                    float t = prng.uniform(spinMin, spinMax);
                    float c = cosf(t);
                    float s = sinf(t);
                    v = Vec2( v[0] * c - v[1] * s ,
                              v[0] * s + v[1] * c );

                    pd.velocity = v;
                    break;
                }
            }
        }

        prng.remix(pd.position[0] * 1e8);
        prng.remix(pd.position[1] * 1e8);

        dynamics[i] = pd;
        appearance[i] = pa;
    }

    totalIntensity = intensityAccumulator;
    if (!numLiveParticles) {
        running = false;
    }
}
