/*
 * Hints of potential, before the bang.
 * Fleeting, square by square.
 * Reaction-diffusion. Precursors to life.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <vector>
#include "lib/effect.h"
#include "lib/prng.h"
#include "lib/texture.h"
#include "lib/noise.h"
#include "lib/camera_flow.h"
#include "lib/rapidjson/document.h"
#include "tree_growth.h"


class Precursor : public Effect
{
public:
    Precursor(const CameraFlowAnalyzer &flow, const rapidjson::Value &config);
    void reseed(unsigned seed);

    virtual void beginFrame(const FrameInfo &f);
    virtual void endFrame(const FrameInfo &f);
    virtual void postProcess(const Vec3& rgb, const PixelInfo& p);
    virtual void shader(Vec3& rgb, const PixelInfo &p) const;
    virtual void debug(const DebugInfo &di);

    TreeGrowth treeGrowth;
    bool isDone;

private:
    float flowScale;
    float flowFilterRate;
    float noiseRate;
    float noiseScale;
    float noiseDepth;
    float brightness;
    float darknessDurationMin;
    float darknessDurationMax;
    float darknessThreshold;

    CameraFlowCapture flow;
    Texture palette;

    float noiseCycle;
    float colorSeed;
    float darknessDurationLimit;
    float darknessDurationCounter;
    float maxActualBrightness;
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline Precursor::Precursor(const CameraFlowAnalyzer& flow, const rapidjson::Value &config)
    : treeGrowth(flow, config["treeGrowth"]),
      flowScale(config["flowScale"].GetDouble()),
      flowFilterRate(config["flowFilterRate"].GetDouble()),
      noiseRate(config["noiseRate"].GetDouble()),
      noiseScale(config["noiseScale"].GetDouble()),
      noiseDepth(config["noiseDepth"].GetDouble()),
      brightness(config["brightness"].GetDouble()),
      darknessDurationMin(config["darknessDurationMin"].GetDouble()),
      darknessDurationMax(config["darknessDurationMax"].GetDouble()),
      darknessThreshold(config["darknessThreshold"].GetDouble()),
      flow(flow),
      palette(config["palette"].GetString())
{
    reseed(42);
}

inline void Precursor::reseed(unsigned seed)
{
    flow.capture(1.0);
    flow.origin();

    treeGrowth.reseed(seed);

    PRNG prng;
    prng.seed(seed);

    noiseCycle = prng.uniform(0, 10);
    colorSeed = prng.uniform(2, 5);
    darknessDurationLimit = prng.uniform(darknessDurationMin, darknessDurationMax);
    darknessDurationCounter = false;
    maxActualBrightness = 0;
    isDone = false;
}

inline void Precursor::beginFrame(const FrameInfo &f)
{
    flow.capture(flowFilterRate);
    treeGrowth.beginFrame(f);
    noiseCycle += f.timeDelta * noiseRate;
    maxActualBrightness = 0;
}

inline void Precursor::endFrame(const FrameInfo &f)
{
    if (maxActualBrightness < darknessThreshold) {
        darknessDurationCounter += f.timeDelta;
    } else {
        darknessDurationCounter = 0;
    }
    isDone = darknessDurationCounter > darknessDurationLimit;
}

inline void Precursor::postProcess(const Vec3& rgb, const PixelInfo& p)
{
    maxActualBrightness = std::max<float>(maxActualBrightness,
        std::max<float>(rgb[0], std::max<float>(rgb[1], rgb[2])));
}

inline void Precursor::debug(const DebugInfo &di)
{
    treeGrowth.debug(di);
    fprintf(stderr, "\t[precursor] noiseCycle = %f\n", noiseCycle);
    fprintf(stderr, "\t[precursor] colorSeed = %f\n", colorSeed);
    fprintf(stderr, "\t[precursor] darknessDurationLimit = %f\n", darknessDurationLimit);
    fprintf(stderr, "\t[precursor] darknessDurationCounter = %f\n", darknessDurationCounter);
    fprintf(stderr, "\t[precursor] maxActualBrightness = %f\n", maxActualBrightness);
    fprintf(stderr, "\t[precursor] darknessThreshold = %f\n", darknessThreshold);
}

inline void Precursor::shader(Vec3& rgb, const PixelInfo &p) const
{
    float n = 1.5 + noiseDepth * fbm_noise3(
        XZ(p.getVec2("gridXY") * noiseScale)
        + flow.model * flowScale
        + Vec3(0, noiseCycle, 0), 4);

    // Lissajous sampling on palette
    rgb = treeGrowth.sampleIntensity(p.point) * brightness *
          palette.sample(0.5 + 0.5 * cos(n),
                         0.5 + 0.5 * sin(n * colorSeed));
}

