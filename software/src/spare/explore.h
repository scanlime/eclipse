/*
 * Movable viewpoint,
 * pre-decided world.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <math.h>
#include <time.h>
#include <stdlib.h>
#include "lib/color.h"
#include "lib/effect.h"
#include "lib/noise.h"
#include "lib/texture.h"
#include "lib/camera_flow.h"


class Explore : public Effect
{
public:
    Explore(CameraFlowAnalyzer& flow, const rapidjson::Value &config)
        : flowScale(config["flowScale"].GetDouble()),
          modelScale(config["modelScale"].GetDouble()),
          startRadius(config["startRadius"].GetDouble()),
          brightness(config["brightness"].GetDouble()),
          flowFilterRate(config["flowFilterRate"].GetDouble()),
          flow(flow),
          map(config["map"].GetString())
    {
        reseed(10);
    }

    float flowScale;
    float modelScale;
    float startRadius;
    float brightness;
    float flowFilterRate;
    CameraFlowCapture flow;
    Texture map;
    Vec2 start;

    void reseed(unsigned seed)
    {
        PRNG prng;
        prng.seed(seed);
        start = Vec2(0.5, 0.5) + prng.circularVector() * startRadius;
    }

    virtual void beginFrame(const FrameInfo &f)
    {
        flow.capture(flowFilterRate);
    }

    virtual void shader(Vec3& rgb, const PixelInfo &p) const
    {
        Vec3 l = p.point * modelScale + flow.model * flowScale;
        rgb = brightness * map.sample(start + Vec2(l[0], -l[2]));
    }

    virtual void debug(const DebugInfo &d)
    {
        fprintf(stderr, "\t[explore] center = (%f, %f)\n",
            start[0] + flow.model[0] * flowScale,
            start[1] + flow.model[2] * flowScale);
    }
};
