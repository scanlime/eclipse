/*
 * Complex particle system.
 * Basic rules give order to things.
 * Sometimes too rigid, like a crystal.
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
#include "narrator.h"


class OrderParticles : public ParticleEffect
{
public:
    OrderParticles(CameraFlowAnalyzer& flow, const rapidjson::Value &config);
    void reseed(unsigned seed);

    virtual void beginFrame(const FrameInfo &f);
    virtual void shader(Vec3& rgb, const PixelInfo& p) const;
    virtual void debug(const DebugInfo &di);

    Texture palette;
    int symmetry;
    float colorCycle;
    float baseAngle;

private:
    unsigned numParticles;
    float centeringGain;
    float flowFilterRate;
    float flowScale;
    float flowLightAngleRate;
    float flowColorCycleRate;
    float relativeSize;
    float intensity;
    float brightness;
    float stepSize;
    float seedRadius;
    float interactionSize;
    float colorRate;
    float angleGainRate;
    float angleGainCenter;
    float angleGainVariation;

    CameraFlowCapture flow;

    unsigned seed;
    float timeDeltaRemainder;

    // Calculated per-frame
    Vec3 lightVec;
    float lightAngle;
    float angleGain;
    Vec3 centerPosition;

    void runStep(const FrameInfo &f);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline OrderParticles::OrderParticles(CameraFlowAnalyzer& flow, const rapidjson::Value &config)
    : palette(config["palette"].GetString()),
      numParticles(config["numParticles"].GetUint()),
      centeringGain(config["centeringGain"].GetDouble()),
      flowFilterRate(config["flowFilterRate"].GetDouble()),
      flowScale(config["flowScale"].GetDouble()),
      flowLightAngleRate(config["flowLightAngleRate"].GetDouble()),
      relativeSize(config["relativeSize"].GetDouble()),
      intensity(config["intensity"].GetDouble()),
      brightness(config["brightness"].GetDouble()),
      stepSize(config["stepSize"].GetDouble()),
      seedRadius(config["seedRadius"].GetDouble()),
      interactionSize(config["interactionSize"].GetDouble()),
      colorRate(config["colorRate"].GetDouble()),
      angleGainRate(config["angleGainRate"].GetDouble()),
      angleGainCenter(config["angleGainCenter"].GetDouble()),
      angleGainVariation(config["angleGainVariation"].GetDouble()),
      flow(flow),
      timeDeltaRemainder(0)
{
    reseed(42);
}

inline void OrderParticles::reseed(unsigned seed)
{
    flow.capture(1.0);
    flow.origin();

    symmetry = 1000;
    lightAngle = 0;

    appearance.resize(numParticles);

    PRNG prng;
    prng.seed(seed);
    this->seed = seed;

    colorCycle = prng.uniform(0, 1000);

    for (unsigned i = 0; i < appearance.size(); i++) {
        Vec2 p = prng.ringVector(1e-4, seedRadius);
        appearance[i].point = Vec3(p[0], 0, p[1]);
    }

    buildIndex();
}

inline void OrderParticles::beginFrame(const FrameInfo &f)
{    
    flow.capture(flowFilterRate);
    flow.origin();

    float t = f.timeDelta + timeDeltaRemainder;
    int steps = t / stepSize;
    timeDeltaRemainder = t - steps * stepSize;

    // Particle appearance
    for (unsigned i = 0; i < appearance.size(); i++) {
        appearance[i].intensity = intensity;
        appearance[i].radius = f.modelRadius * relativeSize;

        // Viewpoint adjustment
        appearance[i].point += flow.model * flowScale;
    }

    while (steps > 0) {
        runStep(f);
        steps--;
    }

    // Lighting
    colorCycle += flow.model[2] * flowColorCycleRate + f.timeDelta * colorRate;
    lightAngle += flow.model[0] * flowLightAngleRate;
    lightVec = Vec3(sin(lightAngle), 0, cos(lightAngle));

    // Angular speed and direction
    angleGain = angleGainCenter + angleGainVariation *
        fbm_noise2(colorCycle * angleGainRate, seed * 5e-7, 2);

    ParticleEffect::beginFrame(f);
}

inline void OrderParticles::runStep(const FrameInfo &f)
{
    // Calculate a new center position while we're here
    Vec3 centerAccumulator(0, 0, 0);

    // Particle interactions
    for (unsigned i = 0; i < appearance.size(); i++) {
        ParticleAppearance pa = appearance[i];

        // Slide toward center of model uniformly
        pa.point -= centerPosition * centeringGain;

        ResultSet_t hits;
        float searchRadius = interactionSize * f.modelRadius;
        index.radiusSearch(hits, pa.point, searchRadius);

        for (unsigned i = 0; i < hits.size(); i++) {
            if (hits[i].first <= i) {
                // Only count each interaction once
                continue;
            }

            // Check distance
            ParticleAppearance &hit = appearance[hits[i].first];
            float q2 = hits[i].second / sq(searchRadius);
            if (q2 < 1.0f) {
                // These particles influence each other
                Vec3 d = hit.point - pa.point;

                // Angular 'snap' force, operates at a distance
                float angle = atan2(d[2], d[0]);
                const float angleIncrement = 2 * M_PI / symmetry;
                float snapAngle = roundf(angle / angleIncrement) * angleIncrement;
                float angleDelta = fabsf(snapAngle - angle);

                // Spin perpendicular to 'd'
                Vec3 da = angleGain * angleDelta * Vec3( d[2], 0, -d[0] );

                da *= kernel2(q2);
                pa.point += da;
                hit.point -= da;
            }
        }

        centerAccumulator += pa.point;
        appearance[i] = pa;
    }

    centerPosition = appearance.size() ? centerAccumulator / appearance.size() : Vec3(0,0,0);
}

inline void OrderParticles::debug(const DebugInfo &di)
{
    fprintf(stderr, "\t[order-particles] symmetry = %d\n", symmetry);
    fprintf(stderr, "\t[order-particles] colorCycle = %f\n", colorCycle);
    fprintf(stderr, "\t[order-particles] lightAngle = %f\n", lightAngle);
    fprintf(stderr, "\t[order-particles] center = (%f, %f, %f)\n", centerPosition[0], centerPosition[1], centerPosition[2]);
    ParticleEffect::debug(di);
}

inline void OrderParticles::shader(Vec3& rgb, const PixelInfo& p) const
{
    // Metaball-style shading with lambertian diffuse lighting and an image-based color palette

    float intensity = sampleIntensity(p.point);
    Vec3 gradient = sampleIntensityGradient(p.point);
    float gradientMagnitude = len(gradient);
    Vec3 normal = gradientMagnitude ? (gradient / gradientMagnitude) : Vec3(0, 0, 0);
    float lambert = 0.6f * std::max(0.0f, dot(normal, lightVec));
    float ambient = 1.0f;

    rgb = (brightness * (ambient + lambert)) * 
        palette.sample(0.5 + 0.5 * sinf(colorCycle), intensity);
}