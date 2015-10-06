#pragma once

#include "lib/effect.h"

class DarknessEffect : public Effect
{
public:
    virtual void shader(Vec3& rgb, const PixelInfo &p) const
    {
        rgb = Vec3(0,0,0);
    }
};
