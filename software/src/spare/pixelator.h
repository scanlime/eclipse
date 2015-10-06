/*
 * Rendering adaptor.
 * Make pixels out of glass blocks.
 * Pixels have more nuance than RGB; angle, contrast, texture.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include "lib/noise.h"
#include "lib/effect.h"


class Pixelator : public Effect {
public:
    struct PixelAppearance {
        Vec3 color;
        float contrast;
        float angle;
        float noise;
    };

    Pixelator();

    void clear();
    unsigned width() const;
    unsigned height() const;
    unsigned pixelIndex(int x, int y) const;
    PixelAppearance& pixelAppearance(int x, int y);
    const PixelAppearance& pixelAppearance(int x, int y) const;

    virtual void beginFrame(const FrameInfo& f);
    virtual void shader(Vec3& rgb, const PixelInfo& p) const;
    virtual void debug(const DebugInfo& d);

private:
    static constexpr float noiseRate = 0.7;
    static constexpr float noiseScale = 0.5;

    PixelAppearance nullAppearance;
    std::vector<PixelAppearance> appearanceBuffer;
    unsigned bufferWidth, bufferHeight;
    float noiseZ;
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline Pixelator::Pixelator()
    : bufferWidth(0), bufferHeight(0), noiseZ(0)
{
    clear();
}

inline void Pixelator::clear()
{
    memset(&appearanceBuffer[0], 0, appearanceBuffer.size() * sizeof appearanceBuffer[0]);
}

inline unsigned Pixelator::width() const
{
    return bufferWidth;
}

inline unsigned Pixelator::height() const
{
    return bufferHeight;
}

inline unsigned Pixelator::pixelIndex(int x, int y) const
{
    if (x < 0 || x >= bufferWidth || y < 0 || y >= bufferHeight) {
        return (unsigned)-1;
    }
    return x + y * bufferWidth;
}

inline Pixelator::PixelAppearance& Pixelator::pixelAppearance(int x, int y)
{
    unsigned i = pixelIndex(x, y);
    if (i < appearanceBuffer.size()) {
        return appearanceBuffer[i];
    } else {
        return nullAppearance;
    }
}

inline const Pixelator::PixelAppearance& Pixelator::pixelAppearance(int x, int y) const
{
    unsigned i = pixelIndex(x, y);
    if (i < appearanceBuffer.size()) {
        return appearanceBuffer[i];
    } else {
        return nullAppearance;
    }
}

inline void Pixelator::beginFrame(const FrameInfo& f)
{
    noiseZ = fmodf(noiseZ + f.timeDelta * noiseRate, 4096.0f);

    // Resize buffer if necessary

    int newWidth = 0, newHeight = 0;

    for (unsigned i = 0; i < f.pixels.size(); i++) {
        if (f.pixels[i].isMapped()) {
            Vec2 gridXY = f.pixels[i].getVec2("gridXY");
            newWidth = std::max<int>(newWidth, gridXY[0] + 1);
            newHeight = std::max<int>(newHeight, gridXY[1] + 1);
        }
    }

    if (newWidth != bufferWidth || newHeight != bufferHeight) {
        bufferWidth = newWidth;
        bufferHeight = newHeight;
        appearanceBuffer.resize(newWidth * newHeight);
        clear();
    }
}

inline void Pixelator::shader(Vec3& rgb, const PixelInfo& p) const
{
    Vec2 gridXY = p.getVec2("gridXY");
    float blockAngle = p.getNumber("blockAngle");
    const PixelAppearance &a = pixelAppearance(gridXY[0], gridXY[1]);

    rgb = a.color * (1.0f
        + a.contrast * cosf(a.angle + blockAngle)
        + a.noise * (0.5 + fbm_noise3( p.point[0] * noiseScale,
                                       p.point[2] * noiseScale,
                                       noiseZ,
                                       3 )));
}

inline void Pixelator::debug(const DebugInfo& d)
{
    fprintf(stderr, "\t[pixelator] %d x %d grid\n", width(), height());
}
