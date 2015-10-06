/*
 * LED Effect delay-line unit. Allows you to sample from the recent past.
 *
 * Copyright (c) 2014 Micah Elizabeth Scott <micah@scanlime.org>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <vector>
#include "effect.h"


class EffectTap : public Effect {
public:
    EffectTap();
    void setEffect(Effect *next);

    struct Frame {
        // Beginning of last frame -> beginning of this frame.
        // If zero, this slot is ununitialized.
        float timeDelta;

        std::vector<Vec3> colors;

        Vec3 averageColor(const PixelInfoVec& pixels) const;
    };

    void resizeBuffer(unsigned numFrames);

    // Look up the frame that was happening 'age' seconds in the past.
    // If that's beyond the end of our buffer, returns NULL.
    const Frame* get(float age) const;

    virtual void shader(Vec3& rgb, const PixelInfo& p) const;
    virtual void postProcess(const Vec3& rgb, const PixelInfo& p);
    virtual void beginFrame(const FrameInfo& f);
    virtual void endFrame(const FrameInfo& f);
    virtual void debug(const DebugInfo& d);

private:
    Effect *next;
    std::vector<Frame> fifo;
    unsigned fifoCurrent;   // Index of the slot we're currently filling
    unsigned fifoValid;     // Number of slots in 'fifo' that have valid data
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline EffectTap::EffectTap()
    : next (0)
{
    // Default buffer size, quite large.
    resizeBuffer(512);
}

inline void EffectTap::setEffect(Effect *next)
{
    this->next = next;
}

inline void EffectTap::resizeBuffer(unsigned numFrames)
{
    fifo.resize(numFrames);
    fifoCurrent = 0;
    fifoValid = 0;
}

inline void EffectTap::shader(Vec3& rgb, const PixelInfo& p) const
{
    next->shader(rgb, p);
}

inline void EffectTap::postProcess(const Vec3& rgb, const PixelInfo& p)
{
    next->postProcess(rgb, p);
    fifo[fifoCurrent].colors[p.index] = rgb;
}

inline void EffectTap::beginFrame(const FrameInfo& f)
{
    unsigned c = (fifoCurrent + 1) % fifo.size();
    fifo[c].colors.resize(f.pixels.size());
    fifo[c].timeDelta = f.timeDelta;
    fifoCurrent = c;

    next->beginFrame(f);
}

inline void EffectTap::endFrame(const FrameInfo& f)
{
    next->endFrame(f);

    // Keep track of the furthest into the FIFO that we've populated
    fifoValid = std::max(fifoValid, fifoCurrent + 1);
}

inline void EffectTap::debug(const DebugInfo& d)
{
    next->debug(d);
}

inline const EffectTap::Frame* EffectTap::get(float age) const
{
    // Assume that the current time is exactly the moment when the
    // current frame began. An age of exactly zero would refer
    // to the current frame. The current frame's own timeDelta is
    // how long it was between the beginning of the last frame and
    // the beginning of the current frame. So if 'age' is less than
    // the current frame's time delta it refers to the previous frame.

    unsigned firstIndex = fifoCurrent;
    unsigned index = firstIndex;

    do {

        if (age <= 0) {
            // Success
            return &fifo[index];
        }

        age -= fifo[index].timeDelta;
        index = (index ? index : fifo.size()) - 1;

    } while (index < fifoValid && index != firstIndex && fifo[index].timeDelta != 0.0f);
    return NULL;
}

inline Vec3 EffectTap::Frame::averageColor(const PixelInfoVec& pixels) const
{
    Vec3 accumulator = Vec3(0,0,0);
    unsigned total = 0;

    for (unsigned i = 0; i < pixels.size(); i++) {
        if (pixels[i].isMapped() && i < colors.size()) {
            total++;
            accumulator += colors[i];
        }
    }

    return total ? accumulator * (1.0f / total) : accumulator;
}
