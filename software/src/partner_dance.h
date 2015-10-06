/*
 * Two entities, exploring ways to interact.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <math.h>
#include <vector>
#include "lib/effect.h"
#include "lib/particle.h"
#include "lib/texture.h"
#include "lib/noise.h"


class PartnerDance : public ParticleEffect
{
public:
    PartnerDance(CameraFlowAnalyzer& flow, const rapidjson::Value &config);
    void reseed(uint32_t seed);

    virtual void beginFrame(const FrameInfo &f);
    virtual void shader(Vec3& rgb, const PixelInfo& p) const;
    virtual void debug(const DebugInfo& d);

    Texture palette;

private:
    static constexpr unsigned numDancers = 2;

    unsigned particlesPerDancer;
    unsigned numParticles;
    float stepSize;
    float noiseRate;
    float radius;
    float radiusScale;
    float intensityScale;
    float maxIntensity;
    float minIntensity;
    float targetRadius;
    float interactionRadius;
    float jitterRate;
    float jitterStrength;
    float jitterScale;
    float flowScale;
    float brightness;
    float targetGain;
    float dampingRate;
    float initialDamping;
    float targetSpin;
    float interactionRate;
    float positionFuzz;
    float separationRadius;

    struct ParticleDynamics {
        Vec2 position;
        Vec2 velocity;
    };

    CameraFlowCapture flow;

    std::vector<ParticleDynamics> dynamics;
    float timeDeltaRemainder;
    float noiseCycle;
    Vec2 target;
    float damping;

    void resetParticle(ParticleDynamics &pd, PRNG &prng, unsigned dancer) const;
    void runStep(const FrameInfo &f);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline PartnerDance::PartnerDance(CameraFlowAnalyzer& flow, const rapidjson::Value &config)
    : palette(config["palette"].GetString()),
      particlesPerDancer(config["particlesPerDancer"].GetUint()),
      numParticles(particlesPerDancer * numDancers),
      stepSize(config["stepSize"].GetDouble()),
      noiseRate(config["noiseRate"].GetDouble()),
      radius(config["radius"].GetDouble()),
      radiusScale(config["radiusScale"].GetDouble()),
      intensityScale(config["intensityScale"].GetDouble()),
      maxIntensity(config["maxIntensity"].GetDouble()),
      minIntensity(config["minIntensity"].GetDouble()),
      targetRadius(config["targetRadius"].GetDouble()),
      interactionRadius(config["interactionRadius"].GetDouble()),
      jitterRate(config["jitterRate"].GetDouble()),
      jitterStrength(config["jitterStrength"].GetDouble()),
      jitterScale(config["jitterScale"].GetDouble()),
      flowScale(config["flowScale"].GetDouble()),
      brightness(config["brightness"].GetDouble()),
      targetGain(config["targetGain"].GetDouble()),
      dampingRate(config["dampingRate"].GetDouble()),
      initialDamping(config["initialDamping"].GetDouble()),
      targetSpin(config["targetSpin"].GetDouble()),
      interactionRate(config["interactionRate"].GetDouble()),
      positionFuzz(config["positionFuzz"].GetDouble()),
      separationRadius(config["separationRadius"].GetDouble()),
      flow(flow),
      timeDeltaRemainder(0)
{
    reseed(42);
}

inline void PartnerDance::reseed(uint32_t seed)
{
    flow.capture(1.0);
    flow.origin();

    PRNG prng;
    prng.seed(seed);

    noiseCycle = prng.uniform(0, 1000);
    damping = initialDamping;

    appearance.resize(numParticles);
    dynamics.resize(numParticles);

    ParticleAppearance *pa = &appearance[0];
    ParticleDynamics *pd = &dynamics[0]; 

    for (unsigned dancer = 0; dancer < numDancers; dancer++) {
        for (unsigned i = 0; i < particlesPerDancer; i++, pa++, pd++) {

            resetParticle(*pd, prng, dancer);

            pa->color = dancer ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
        }
    }
}

inline void PartnerDance::beginFrame(const FrameInfo &f)
{    
    noiseCycle += f.timeDelta * noiseRate;
    damping += f.timeDelta * dampingRate;

    float t = f.timeDelta + timeDeltaRemainder;
    int steps = t / stepSize;
    timeDeltaRemainder = t - steps * stepSize;

    // Update orbit angles
    float angle1 = noiseCycle;
    float angle2 = noiseCycle * 0.2f;
    float angle3 = noiseCycle * 0.7f;

    target[0] = targetRadius * sinf(angle1);
    target[1] = targetRadius * cosf(angle1 + sin(angle2));

    // Update all particle radii
    float r = radius + radiusScale * sinf(angle3);
    for (unsigned i = 0; i < appearance.size(); i++) {
        appearance[i].radius = r;
    }

    // Build first index
    ParticleEffect::beginFrame(f);

    while (steps > 0) {
        runStep(f);
        steps--;

        // Rebuild index
        ParticleEffect::beginFrame(f);
    }
}

inline void PartnerDance::debug(const DebugInfo& d)
{
    fprintf(stderr, "\t[partner-dance] numParticles = %d\n", numParticles);
    fprintf(stderr, "\t[partner-dance] radius = %f\n", appearance[0].radius);
    fprintf(stderr, "\t[partner-dance] noiseCycle = %f\n", noiseCycle);
    fprintf(stderr, "\t[partner-dance] damping = %f\n", damping);
    fprintf(stderr, "\t[partner-dance] flow.model = [%f, %f]\n", flow.model[0], flow.model[2]);
    ParticleEffect::debug(d);
}

inline void PartnerDance::runStep(const FrameInfo &f)
{
    ParticleAppearance *pa = &appearance[0];
    ParticleDynamics *pd = &dynamics[0]; 

    PRNG prng;
    prng.seed(42);

    flow.capture();

    for (unsigned dancer = 0; dancer < numDancers; dancer++) {
        for (unsigned i = 0; i < particlesPerDancer; i++, pa++, pd++) {

            prng.remix(pd->position[0] * 1e8);
            prng.remix(pd->position[1] * 1e8);

            Vec2 fPos = pd->position + Vec2(flow.model[0], flow.model[2]) * flowScale;
            Vec2 disparity = target - fPos;
            Vec2 normal = Vec2(disparity[1], -disparity[0]);
            Vec2 v = pd->velocity;

            // Single-particle velocity updates
            v *= 1.0 - damping;
            v += disparity * targetGain;
            v += normal * targetSpin;

            // Particle interactions
            ResultSet_t hits;
            index.radiusSearch(hits, pa->point, interactionRadius);

            for (unsigned i = 0; i < hits.size(); i++) {
                unsigned hitDancer = hits[i].first / particlesPerDancer;
                if (hitDancer == dancer) {
                    // Only interact with other dancers
                    continue;
                }

                // Check distance
                float q2 = hits[i].second / sq(interactionRadius);
                if (q2 >= 1.0f) {
                    continue;
                }
                float k = dancer ? kernel2(q2) : -kernel(q2);

                // Spin force, normal to the angle between us
                ParticleDynamics &other = dynamics[hits[i].first];
                Vec2 d = other.position - pd->position;
                Vec2 normal = Vec2(d[1], -d[0]);
                normal /= len(normal);

                v += interactionRate * k * normal;
            }

            pd->velocity = v;
            pd->position += v;

            pa->point = Vec3(fPos[0], 0, fPos[1]);
            pa->intensity = std::min(float(maxIntensity), intensityScale * len(v));

            if (pa->intensity < minIntensity) {
                resetParticle(*pd, prng, dancer);
            }
        }
    }
}

inline void PartnerDance::resetParticle(ParticleDynamics &pd, PRNG &prng, unsigned dancer) const
{
    pd.velocity = Vec2(0, 0);
    pd.position = prng.circularVector() * positionFuzz + (dancer ? Vec2(separationRadius, 0) : Vec2(-separationRadius, 0));
}

inline void PartnerDance::shader(Vec3& rgb, const PixelInfo& p) const
{
    Vec3 jitter = Vec3(
        fbm_noise3(noiseCycle * jitterRate, p.point[0] * jitterScale, p.point[2] * jitterScale, 4) * jitterStrength,
        fbm_noise3(noiseCycle * jitterRate, p.point[0] * jitterScale, p.point[2] * jitterScale, 4) * jitterStrength,
        0);

    // Use 'color' to encode contributions from both partners
    Vec3 c = sampleColor(p.point) + jitter;

    // 2-dimensional palette lookup
    rgb = brightness * palette.sample(c[0], c[1]);
}
