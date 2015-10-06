/*
 * Perfectly solid color everywhere. Boring on its own, but it makes a pivot point for other effects.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <stdio.h>
#include "lib/effect.h"
#include "lib/effect_tap.h"


class ColorField : public Effect {
public:
    ColorField();

    void set(Vec3 color);
    void set(const EffectRunner &runner, const EffectTap &tap, float age = 0.0f);

    virtual void shader(Vec3& rgb, const PixelInfo& p) const;
    virtual void debug(const DebugInfo &di);

private:
    Vec3 color;
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline ColorField::ColorField()
    : color(0, 0, 0)
{}

inline void ColorField::set(Vec3 color)
{
    this->color = color;
}

inline void ColorField::set(const EffectRunner &runner, const EffectTap &tap, float age)
{
    const EffectTap::Frame *f = tap.get(0);
    if (f) {
        set(f->averageColor(runner.getPixelInfo()));
    }
}

inline void ColorField::shader(Vec3& rgb, const PixelInfo& p) const
{
    rgb = color;
}

inline void ColorField::debug(const DebugInfo &di)
{
    fprintf(stderr, "\t[color-field] color = (%f, %f, %f)\n", color[0], color[1], color[2]);
}
