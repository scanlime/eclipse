/*
 * Grid, leading to emergent order.
 * Langton's ants. Chunky pixels.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include "pixelator.h"
#include "dark_followers.h"
#include "lib/prng.h"
#include "lib/texture.h"


class Ants : public Pixelator {
public:
    Ants();
    void reseed(unsigned seed);

    virtual void beginFrame(const FrameInfo& f);
    virtual void shader(Vec3& rgb, const PixelInfo& p) const;

    float antStepRate;
    float antStepRateDelta;
    float colorParam;

private:
    static constexpr unsigned numAnts = 1;
    static constexpr float stepRate = 200;
    static constexpr float noiseMax = 1.5;
    static constexpr float noiseFadeRate = 0.007;
    static constexpr float colorFadeRate = 0.015;
    static constexpr float angleRate = 10.0;
    static constexpr float colorParamRate = 0.29;
    static constexpr float spiralRate = 0.08;
    static constexpr float pushRadius = 0.3;
    static constexpr float pushAmount = 0.0006;
    static constexpr float sinkRate = 0.0001;

    struct Ant {
        int x, y, direction;
        void reseed(PRNG &prng);
        void update(Ants &world);
    };

    DarkFollowers darkness;
    Ant ants[numAnts];
    float antTimeDeltaRemainder;
    float timeDeltaRemainder;
    std::vector<unsigned> state;
    std::vector<Vec3> targetColors;
    std::vector<Vec3> pushVectors;
    Texture palette;

    void runStep(const FrameInfo &f);
    void filterColor(int x, int y);
    Vec3 targetColorForState(unsigned st);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


static inline int umod(int a, int b)
{
    int c = a % b;
    if (c < 0) {
        c += b;
    }
    return c;
}

inline Ants::Ants()
    : antTimeDeltaRemainder(0), 
      timeDeltaRemainder(0),
      palette("data/succulent-palette.png")
{
    reseed(42);
}

inline void Ants::reseed(unsigned seed)
{
    antStepRate = 1;
    antStepRateDelta = 0;

    clear();
    state.clear();
    targetColors.clear();
    pushVectors.clear();

    PRNG prng;
    prng.seed(seed);
    darkness.reseed(prng.uniform32());
    for (unsigned i = 0; i < numAnts; i++) {
        ants[i].reseed(prng);
    }

    colorParam = prng.uniform(0, M_PI * 2);
}

inline void Ants::shader(Vec3& rgb, const PixelInfo& p) const
{
    Pixelator::shader(rgb, p);
    rgb += darkness.sampleColor(p.point);
}

inline void Ants::beginFrame(const FrameInfo& f)
{
    Pixelator::beginFrame(f);
    darkness.beginFrame(f);

    if (state.size() != width() * height()) {
        // Resize and erase parallel arrays, now that we know size
        state.resize(width() * height());
        targetColors.resize(width() * height());
        pushVectors.resize(width() * height());
        memset(&state[0], 0, state.size() * sizeof state[0]);
        memset(&targetColors[0], 0, targetColors.size() * sizeof targetColors[0]);
        memset(&pushVectors[0], 0, pushVectors.size() * sizeof pushVectors[0]);
    }

    colorParam += f.timeDelta * colorParamRate;

    // Fixed timestep
    float t = f.timeDelta + antTimeDeltaRemainder;
    int steps = t * antStepRate;
    antTimeDeltaRemainder = t - steps / antStepRate;
    antStepRate += steps * antStepRateDelta;
    while (steps > 0) {
        for (unsigned i = 0; i < numAnts; i++) {
            ants[i].update(*this);
        }
        steps--;
    }

    t = f.timeDelta + timeDeltaRemainder;
    steps = t * stepRate;
    timeDeltaRemainder = t - steps / stepRate;
    while (steps > 0) {
        runStep(f);
        steps--;
    }

    // Visual updates, at least once per device frame
    for (unsigned y = 0; y < height(); y++) {
        for (unsigned x = 0; x < width(); x++) {
            filterColor(x, y);
        }
    }
}

inline void Ants::filterColor(int x, int y)
{
    PixelAppearance &a = pixelAppearance(x, y);
    Vec3 targetColor = targetColors[pixelIndex(x, y)];

    a.color += (targetColor - a.color) * colorFadeRate;
    a.noise *= 1.0 - noiseFadeRate;
}

inline Vec3 Ants::targetColorForState(unsigned st)
{
    static float brightness[] = { 0, 2.5, 0.75 };
    float r = 0.65 / (1 + colorParam * spiralRate);
    float t = colorParam;
    float br = st < 3 ? brightness[st] : 0;
    return palette.sample(cosf(t) * r + 0.5,
                          sinf(t) * r + 0.5) * br;
}

inline void Ants::Ant::reseed(PRNG &prng)
{
    x = prng.uniform(1, 4);
    y = prng.uniform(1, 10);
    direction = prng.uniform(0, 10);
}

inline void Ants::Ant::update(Ants &world)
{
    x = umod(x, world.width());
    y = umod(y, world.height());

    // Also update color here, in case our step rate is much faster than
    // the device frame rate
    world.filterColor(x, y);

    PixelAppearance &a = world.pixelAppearance(x, y);
    unsigned &st = world.state[world.pixelIndex(x, y)];
    Vec3 &pushVector = world.pushVectors[world.pixelIndex(x, y)];

    if (st == 1) {
        direction--;
        st = 2;

    } else {
        direction++;
        st = 1;

        a.angle += angleRate / world.antStepRate;
        a.contrast = 0.4;
        a.noise = noiseMax;
    }

    world.targetColors[world.pixelIndex(x, y)] = world.targetColorForState(st);

    direction = umod(direction, 4);
    switch (direction) {
        case 0: x++; pushVector = Vec3(pushAmount,0,0); break;
        case 1: y++; pushVector = Vec3(0,0,pushAmount); break;
        case 2: x--; pushVector = Vec3(-pushAmount,0,0); break;
        case 3: y--; pushVector = Vec3(0,0,-pushAmount); break;
    }
}

inline void Ants::runStep(const FrameInfo &f)
{
    // Push all dark dots around, according to the trail our ant leaves
    for (unsigned pixel = 0; pixel < f.pixels.size(); pixel++) {
        const PixelInfo &p = f.pixels[pixel];
        if (p.isMapped()) {
            Vec2 gridXY = p.getVec2("gridXY");
            darkness.push(p.point, pushRadius, pushVectors[pixelIndex(gridXY[0], gridXY[1])]);
        }
    }

    // Sink toward the middle
    darkness.snap(Vec3(0,0,0), sinkRate);
}
