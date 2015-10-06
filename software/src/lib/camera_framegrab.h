/*
 * Camera frame-grabber utility.
 *
 * The "Camera" interface represents a low-level analog video
 * capture hook. That level is where we build our interactivity
 * algorithms, to get the lowest latency. But we still want to
 * see the camera's stream sometimes! This class assembles single
 * frames of video, saving them as JPEG files.
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

#include <stdio.h>
#include <string>
#include "jpge.h"
#include "camera.h"


class CameraFramegrab {
public:
    CameraFramegrab();

    // Start saving the next complete frame to the indicated file, as a JPEG
    void begin(const char *filename);

    // Cancel a pending grab.
    void cancel();

    // Is a framegrab in progress? A framegrab must see an entire frame of video go by.
    bool isGrabbing();

    // Process another chunk of video data. Returns 'true' as long as a
    // framegrab is in progress (it wants more data).
    bool process(const Camera::VideoChunk &chunk);

    // Returns raw capture-buffer in UYVY format.
    const uint8_t *getUYVY() const;

private:
    uint8_t frame[Camera::kPixels * 2];
    uint8_t rgb[Camera::kPixels * 3];

    std::string grabFile;
    unsigned grabLine;
    unsigned grabVBL;

    void finishGrab();
};


class CameraPeriodicFramegrab {
public:
    CameraPeriodicFramegrab(const char *name = "frame", float interval = 1.0f, int firstIndex = 1);

    CameraFramegrab grab;

    void timeStep(float ts);
    void process(const Camera::VideoChunk &chunk);

private:
    float timer, interval;
    int index;
    const char *name;
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline CameraFramegrab::CameraFramegrab()
    : grabLine(0), grabVBL(0)
{}

inline void CameraFramegrab::begin(const char *filename)
{
    cancel();
    grabFile.assign(filename);
}

inline void CameraFramegrab::cancel()
{
    grabFile.clear();

    // Initialize line for VBL detector. Zero will require
    // at least two samples to determine if 'line' has jumped
    // backwards.
    grabLine = 0;

    // Count VBLs (Vertical Blanking events) to determine when
    // we've seen an entire frame. We need to see the VBLs
    // immediately preceeding and following a frame in order to
    // capture it.
    grabVBL = 0;
}

inline bool CameraFramegrab::isGrabbing()
{
    return !grabFile.empty();
}

inline const uint8_t *CameraFramegrab::getUYVY() const
{
    return frame;
}

inline bool CameraFramegrab::process(const Camera::VideoChunk &chunk)
{
    // Store data in capture buffer. Always do this when process()
    // is called, to keep the getUYVY() results fresh. If the caller
    // wants to avoid this overhead when not grabbing, use an
    // isGrabbing() test first.
    //
    // Note that this will run on the camera thread!

    memcpy(frame + chunk.framebufferOffset(), chunk.data, chunk.byteCount);

    if (!isGrabbing()) {
        return false;
    }

    bool finish = false;

    // Detect vertical blanking events, by watching 'line' go backwards.
    if (chunk.line < grabLine) {
        grabVBL++;

        // If we've seen three VBLs, we have a complete frame.
        // (The two VBLs that bookend an alternate pair of fields.)
        if (grabVBL == 3) {
            finishGrab();
            finish = true;
        }
    }
    grabLine = chunk.line;

    // Still need more data?
    return !finish;
}

inline void CameraFramegrab::finishGrab()
{
    // Convert UYVY to RGB

    const uint8_t *src = frame;
    uint8_t *dest = rgb;

    for (unsigned i = Camera::kPixels / 2; i; --i) {

        int u  = *(src++) - 128;
        int y1 = *(src++) - 16;
        int v  = *(src++) - 128;
        int y2 = *(src++) - 16;

        int r = (v * 91947) >> 16;
        int g = (u * -22544 + v * -46793) >> 16;
        int b = (u * 115999) >> 16;

        *(dest++) = std::max(0, std::min(255, r + y1));
        *(dest++) = std::max(0, std::min(255, g + y1));
        *(dest++) = std::max(0, std::min(255, b + y1));

        *(dest++) = std::max(0, std::min(255, r + y2));
        *(dest++) = std::max(0, std::min(255, g + y2));
        *(dest++) = std::max(0, std::min(255, b + y2));
    }

    // Compress the JPEG
    if (!jpge::compress_image_to_jpeg_file(
            grabFile.c_str(), Camera::kPixelsPerLine, Camera::kLinesPerField * Camera::kFields,
            3, rgb)) {
        fprintf(stderr, "camera: Failed to capture frame to %s\n", grabFile.c_str());
    }

    fprintf(stderr, "camera: captured %s\n", grabFile.c_str());

    // Done
    cancel();
}

inline CameraPeriodicFramegrab::CameraPeriodicFramegrab(const char *name, float interval, int firstIndex)
    : timer(interval), interval(interval), index(firstIndex), name(name)
{}

void CameraPeriodicFramegrab::timeStep(float ts)
{
    timer += ts;
    if (timer > interval) {

        char buffer[1024];
        snprintf(buffer, sizeof buffer, "%s-%04d.jpeg", name, index);
        grab.begin(buffer);

        timer = fmodf(timer, interval);
        index++;
    }
}

void CameraPeriodicFramegrab::process(const Camera::VideoChunk &chunk)
{
    if (grab.isGrabbing()) {
        grab.process(chunk);
    }
}
