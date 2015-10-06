/*
 * Highly dynamic, patterns of flow, expanding outward over the grid
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <vector>
#include "lib/effect.h"
#include "lib/particle.h"
#include "lib/prng.h"
#include "lib/camera_flow.h"
#include "lib/rapidjson/document.h"


class TreeGrowth : public ParticleEffect
{
public:
    TreeGrowth(const CameraFlowAnalyzer &flow, const rapidjson::Value &config);
    void reseed(unsigned seed);

    virtual void beginFrame(const FrameInfo &f);
    virtual void debug(const DebugInfo &di);

    void launch(Vec3 point, Vec3 velocity);

private:
    unsigned maxParticles;
    float flowLaunchScale;
    float flowLaunchSpeed;
    float launchProbability;
    float launchPointNoise;
    float launchVelocityNoise;
    float launchHistoryDepth;
    float stepRate;
    float brightness;
    float particleDuration;
    float particleRampup;
    float gradientPull;
    float ledPull;
    float ledPullRadius;
    float blockPull;
    float blockPullRadius;
    float damping;
    float visibleRadius;
    float outsideMargin;
    float flowFilterRate;
    float spuriousLaunchProbabilityInitial;
    float spuriousLaunchProbabilityRate;
    float spuriousLaunchRadiusMin;
    float spuriousLaunchRadiusMax;
    float spuriousLaunchSpeedMin;
    float spuriousLaunchSpeedMax;

    struct ParticleDynamics {
        Vec3 velocity;
        float time;
    };

    CameraFlowCapture flow;

    std::vector<ParticleDynamics> dynamics;
    PRNG prng;
    float timeDeltaRemainder;
    float spuriousLaunchProbability;

    Vec3 launchPosition(const FrameInfo &f, Vec3 direction) const;
    float particleIntensity(float t) const;
    void runStep(const FrameInfo &f);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline TreeGrowth::TreeGrowth(const CameraFlowAnalyzer& flow, const rapidjson::Value &config)
    : maxParticles(config["maxParticles"].GetUint()),
      flowLaunchScale(config["flowLaunchScale"].GetDouble()),
      flowLaunchSpeed(config["flowLaunchSpeed"].GetDouble()),
      launchProbability(config["launchProbability"].GetDouble()),
      launchPointNoise(config["launchPointNoise"].GetDouble()),
      launchVelocityNoise(config["launchVelocityNoise"].GetDouble()),
      launchHistoryDepth(config["launchHistoryDepth"].GetDouble()),
      stepRate(config["stepRate"].GetDouble()),
      brightness(config["brightness"].GetDouble()),
      particleDuration(config["particleDuration"].GetDouble()),
      particleRampup(config["particleRampup"].GetDouble()),
      gradientPull(config["gradientPull"].GetDouble()),
      ledPull(config["ledPull"].GetDouble()),
      ledPullRadius(config["ledPullRadius"].GetDouble()),
      blockPull(config["blockPull"].GetDouble()),
      blockPullRadius(config["blockPullRadius"].GetDouble()),
      damping(config["damping"].GetDouble()),
      visibleRadius(config["visibleRadius"].GetDouble()),
      outsideMargin(config["outsideMargin"].GetDouble()),
      flowFilterRate(config["flowFilterRate"].GetDouble()),      
      spuriousLaunchProbabilityInitial(config["spuriousLaunchProbabilityInitial"].GetDouble()),      
      spuriousLaunchProbabilityRate(config["spuriousLaunchProbabilityRate"].GetDouble()),      
      spuriousLaunchRadiusMin(config["spuriousLaunchRadiusMin"].GetDouble()),      
      spuriousLaunchRadiusMax(config["spuriousLaunchRadiusMax"].GetDouble()),      
      spuriousLaunchSpeedMin(config["spuriousLaunchSpeedMin"].GetDouble()),      
      spuriousLaunchSpeedMax(config["spuriousLaunchSpeedMax"].GetDouble()),      
      flow(flow),
      timeDeltaRemainder(0)
{
    reseed(42);
}

inline void TreeGrowth::reseed(unsigned seed)
{
    flow.capture(1.0);
    flow.origin();
    prng.seed(seed);

    spuriousLaunchProbability = spuriousLaunchProbabilityInitial;
}

inline float TreeGrowth::particleIntensity(float t) const
{
    // Ramp up, then fade
    return brightness * cos(t * (M_PI/2)) * kernel(1.0f - std::min(1.0f, t / particleRampup));
}

inline void TreeGrowth::beginFrame(const FrameInfo &f)
{
    // Fixed timestep
    float t = f.timeDelta + timeDeltaRemainder;
    int steps = t * stepRate;
    timeDeltaRemainder = t - steps / stepRate;

    while (steps > 0) {
        runStep(f);
        steps--;
    }

    ParticleEffect::beginFrame(f);
}

inline void TreeGrowth::debug(const DebugInfo &di)
{
    fprintf(stderr, "\t[tree-growth] particles = %d\n", (int)appearance.size());
    fprintf(stderr, "\t[tree-growth] motionLength = %f\n", flow.motionLength);
    fprintf(stderr, "\t[tree-growth] instantaneousMotion = %f\n", flow.instantaneousMotion());
    fprintf(stderr, "\t[tree-growth] spuriousLaunchProbability = %f\n", spuriousLaunchProbability);
}

inline void TreeGrowth::launch(Vec3 point, Vec3 velocity)
{
    ParticleAppearance pa;
    ParticleDynamics pd;

    pa.color = Vec3(1,1,1);
    pa.point = point;
    pa.radius = visibleRadius;
    pd.velocity = velocity;
    pd.time = 0;

    appearance.push_back(pa);
    dynamics.push_back(pd);
}

inline Vec3 TreeGrowth::launchPosition(const FrameInfo &f, Vec3 direction) const
{
    // At the edge of the model's AABB, in the indicated direction.

    Vec3 v = direction * f.modelRadius / len(direction);

    if (v[0] > f.modelMax[0]) {
        v[2] *= f.modelMax[0] / v[0];
        v[0] = f.modelMax[0];
    } else if (v[0] < f.modelMin[0]) {
        v[2] *= f.modelMin[0] / v[0];
        v[0] = f.modelMin[0];
    }

    if (v[2] > f.modelMax[2]) {
        v[0] *= f.modelMax[2] / v[2];
        v[2] = f.modelMax[2];
    } else if (v[2] < f.modelMin[2]) {
        v[0] *= f.modelMin[2] / v[2];
        v[2] = f.modelMin[2];
    }

    return v;
}

inline void TreeGrowth::runStep(const FrameInfo &f)
{
    // Launch completely spurious particles; their probability decreases over time.
    // This helps demarcate the transition into this effect, plus it gives us a backup
    // trigger in case our camera input is disabled or broken.
    if (spuriousLaunchProbability > 0) { 
        int launchCount = std::min<int>(maxParticles - appearance.size(),
            prng.uniform(0, 1.0f + spuriousLaunchProbability));
        spuriousLaunchProbability += spuriousLaunchProbabilityRate;

        while (launchCount--) {
            Vec2 p = prng.ringVector(spuriousLaunchRadiusMin, spuriousLaunchRadiusMax); 
            Vec2 v = prng.ringVector(spuriousLaunchSpeedMin, spuriousLaunchSpeedMax);
            launch(Vec3(p[0], 0, p[1]), Vec3(v[0], 0, v[1]));
        }
    }

    // Launch new particles based on flow
    if (flowLaunchScale) {
        flow.capture(flowFilterRate);
        flow.origin();

        int launchCount = std::min<int>(maxParticles - appearance.size(),
            prng.uniform(0, 1.0f + flow.instantaneousMotion() * flowLaunchScale));

        while (launchCount--) {
            launch(launchPosition(f, -flow.model), flow.model * flowLaunchSpeed);
        }
    }

    // Launch new particles based on random parent particles
    if (!appearance.empty()) {
        int launchCount =
            std::max<int>(0,
            std::min<int>(maxParticles - appearance.size(),
                prng.uniform(0, 1.0f + launchProbability)));

        while (launchCount--) {
            int p = std::max<int>(0, appearance.size() - prng.uniform(1, 1 + launchHistoryDepth));
            launch(appearance[p].point + XZ(prng.circularVector() * launchPointNoise),
                   dynamics[p].velocity + XZ(prng.circularVector() * launchVelocityNoise));
        }
    }

    // Iterate through and update all particles, discarding any that have expired
    unsigned j = 0;
    for (unsigned i = 0; i < appearance.size(); i++) {
        ParticleAppearance pa = appearance[i];
        ParticleDynamics pd = dynamics[i];

        // Update simulation

        Vec3 v = pd.velocity * (1.0f - damping);
        Vec3 pt = pa.point;
        float t = pd.time;

        v += gradientPull * sampleIntensityGradient(pa.point);
        pt += v;
        t += 1.0f / stepRate / particleDuration;

        pd.velocity = v;
        pa.point = pt;
        pd.time = t;

        if (pd.time >= 1.0f) {
            // Discard, too old
            continue;
        }

        if (f.distanceOutsideBoundingBox(pt) > outsideMargin * visibleRadius) {
            // Outside bounding box, discard
            continue;
        }

        pa.intensity = particleIntensity(t);

        // Pull toward nearby LEDs, so the particles kinda-follow the grid.

        ResultSet_t hits;
        f.radiusSearch(hits, pa.point, std::max(ledPullRadius, blockPullRadius));
        for (unsigned h = 0; h < hits.size(); h++) {
            const PixelInfo &hit = f.pixels[hits[h].first];
            if (!hit.isMapped()) {
                continue;
            }

            Vec3 v = pd.velocity;

            // Pull toward LED
            Vec3 d = hit.point - pt;
            float q2 = hits[h].second / sq(ledPullRadius);
            if (q2 < 1.0f) {
                v += ledPull * kernel2(q2) * d;
            }

            // Pull toward grid square center
            q2 = hits[h].second / sq(blockPullRadius);
            if (q2 < 1.0f) {
                Vec2 blockXY = hit.getVec2("blockXY");
                Vec2 b = blockPull * kernel2(q2) * blockXY;
                v += Vec3(-b[0], 0, b[1]);
            }

            pd.velocity = v;
        }

        // Write out
        appearance[j] = pa;
        dynamics[j] = pd;
        j++;
    }

    appearance.resize(j);
    dynamics.resize(j);
}
