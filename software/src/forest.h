/*
 * Seeing nothing but trees.
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
#include "lib/sampler.h"


class Forest : public ParticleEffect
{
public:
    Forest(const CameraFlowAnalyzer &flow, const rapidjson::Value &config);
    void reseed(unsigned seed);

    virtual void beginFrame(const FrameInfo &f);
    virtual void debug(const DebugInfo &di);

private:
    struct TreeInfo {
        Vec2 texCoord;
        Vec3 direction;
        Vec3 point;
        unsigned branchState;
    };

    Sampler s;
    CameraFlowCapture flow;
    const rapidjson::Value& config;

    unsigned maxParticles;
    float outsideMargin;
    float flowScale;
    float flowFilterRate;
    float maxIntensity;
    float intensityRate;
    float growthPointsPerSecond;
    Vec3 travelRate;

    Texture palette;
    std::vector<TreeInfo> tree;
    Vec2 newTexCoord;
    float travelAmount;
    float growthAmount;

    void addPoint();
    void updateFlow(const FrameInfo &f);
    Vec3 pointOffset();
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline Forest::Forest(const CameraFlowAnalyzer &flow, const rapidjson::Value &config)
    : s(42),
      flow(flow),
      config(config),
      maxParticles(config["maxParticles"].GetUint()),
      palette(config["palette"].GetString())
    {
 }

inline void Forest::reseed(unsigned seed)
{
    appearance.clear();
    tree.clear();
    flow.capture(1.0);
    flow.origin();
    s = Sampler(seed);
    travelAmount = 0;

    newTexCoord = s.value2D(config["newTexCoord"]);
    outsideMargin = s.value(config["outsideMargin"]);
    flowScale = s.value(config["flowScale"]);
    flowFilterRate = s.value(config["flowFilterRate"]);
    maxIntensity = s.value(config["maxIntensity"]);
    intensityRate = s.value(config["intensityRate"]);
    growthPointsPerSecond = s.value(config["growthPointsPerSecond"]);
    travelRate = s.value3D(config["travelRate"]);
}

inline void Forest::beginFrame(const FrameInfo &f)
{
    growthAmount += growthPointsPerSecond * f.timeDelta;
    while (growthAmount > 1.0f) {
        if (appearance.size() >= maxParticles) {
            growthAmount = 0;
            break;
        }
        addPoint();
        growthAmount -= 1.0f;
    }

    updateFlow(f);

    travelAmount += f.timeDelta;
    ParticleEffect::beginFrame(f);
}

inline Vec3 Forest::pointOffset()
{
    return Vec3(flow.model[0] * flowScale, 0, 0) + travelRate * travelAmount;
}

inline void Forest::updateFlow(const FrameInfo &f)
{
    flow.capture(flowFilterRate);

    unsigned i = 0, j = 0;
    for (; i < appearance.size(); i++) {
        appearance[i].point = tree[i].point + pointOffset();
        appearance[i].intensity = std::min(maxIntensity, appearance[i].intensity + f.timeDelta * intensityRate);

        if (f.distanceOutsideBoundingBox(appearance[i].point) > outsideMargin * appearance[i].radius) {
            // Escaped
            continue;
        }

        if (j != i) {
            appearance[j] = appearance[i];
            tree[j] = tree[i];
        }
        j++;
    }
    appearance.resize(j);
    tree.resize(j);
}

inline void Forest::addPoint()
{
    ParticleAppearance pa;
    TreeInfo ti;

    pa.radius = s.value(config["radius"]);
    pa.intensity = 0;

    int root = std::max<int>(0,
        s.uniform(appearance.size() - s.value(config["historyDepth"]),
                  appearance.size() + s.value(config["newnessBias"])));

    if (root >= (int)appearance.size()) {
        // Start a new tree
        newTexCoord += s.mRandom.circularVector() * s.value(config["walkTexCoord"]);
        ti.texCoord = newTexCoord;
        ti.direction = s.value3D(config["newDirection"]);
        ti.branchState = 0;
        ti.point = s.value3D(config["newPoint"]) - pointOffset();
    } else {
        ti.texCoord = tree[root].texCoord + s.mRandom.circularVector() * s.value(config["deltaTexCoord"]);
        ti.direction = tree[root].direction + s.value3D(config["deltaDirection"]);
        ti.branchState = tree[root].branchState + 1;
        ti.point = tree[root].point + tree[root].direction;
    }

    // Texture sampler already clamps, but this keeps our random walk constrained within the texture bounds.
    ti.texCoord[0] = std::min(1.0f, std::max(0.0f, ti.texCoord[0]));
    ti.texCoord[1] = std::min(1.0f, std::max(0.0f, ti.texCoord[1]));

    pa.color = palette.sample(ti.texCoord);

    appearance.push_back(pa);
    tree.push_back(ti);
}

inline void Forest::debug(const DebugInfo &di)
{
    ParticleEffect::debug(di);
    fprintf(stderr, "\t[forest] numParticles = %d\n", (int)appearance.size());
}
