/*
 * Camera sampler layer: Ways of breaking down camera data into
 * precursors used by our other algorithms.
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

#include "camera.h"


/*
 * This picks out a relatively small number
 * of individual luminance pixels to sample. We use the symmetric
 * solution to the 8 queens problem, in order to use 1/8 the pixels
 * without losing any entire rows or columns.
 *
 * This class acts as a VideoChunk processor that emits callbacks
 * for each new sample it encounters. Video is arriving nearly in
 * real-time, each sample is processed as soon as its Isoc buffer
 * is available.
 */
class CameraSampler8Q
{
public:
    static const int kSamplesPerBlock = 8;
    static const int kBlocksWide = Camera::kPixelsPerLine / kSamplesPerBlock;
    static const int kBlocksHigh = Camera::kLinesPerFrame / kSamplesPerBlock;
    static const int kBlocks = kBlocksWide * kBlocksHigh;
    static const int kSamples = kBlocks * kSamplesPerBlock;

    CameraSampler8Q(const Camera::VideoChunk &chunk);

    /*
     * Look for a sample in the current chunk. Returns 'true' if the sample
     * was found, and sets 'index' and 'luminance'. Returns 'false' if no more
     * samples are in this chunk.
     */
    bool next(unsigned &index, uint8_t &luminance);

    // Calculate the position of a particular sample
    static int sampleX(unsigned index);
    static int sampleY(unsigned index);

    // Calculate the block position of a particular sample
    static int blockX(unsigned index);
    static int blockY(unsigned index);
    static unsigned blockIndex(unsigned index);

    static unsigned x8q(unsigned y);

private:
    Camera::VideoChunk iter;
    unsigned y;
    unsigned yIndex;
};


/*
 * Buffer every sampled luminance value, using the 8Q grid
 */
class CameraLuminanceBuffer
{
public:
    CameraLuminanceBuffer();
    void process(const Camera::VideoChunk &chunk);
    uint8_t buffer[CameraSampler8Q::kSamples];
};


/*
 * Sample every luminance pixel, and use them to update a Sobel edge detection filter.
 * The sobel filter is at full resolution, but the magnitude results are indexed
 * using the CameraSampler8Q grid.
 */
class CameraSamplerSobel
{
public:
    CameraSamplerSobel();
    void process(const Camera::VideoChunk &chunk);

    // Diff buffer for raw video
    uint8_t luminance[Camera::kPixels];

    // Diff buffer for sobel magnitude (XY)
    float sobelXY[CameraSampler8Q::kSamples];

    // Motion filter, boosted when sobelXY changes
    float motion[CameraSampler8Q::kSamples];

private:
    static const float kMotionFilterGain = 1e-2;

    struct {
        // Layout of buffers such that we don't need to bounds-check while updating sobel filter
        int padding1[Camera::kPixelsPerLine * 2];
        int sobelX[Camera::kPixels];
        int sobelY[Camera::kPixels];
        int padding2[Camera::kPixelsPerLine * 2];
    };
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline CameraSampler8Q::CameraSampler8Q(const Camera::VideoChunk &chunk)
    : iter(chunk),
      y(iter.line * Camera::kFields + iter.field),
      yIndex(y * kBlocksWide)
{}

inline unsigned CameraSampler8Q::x8q(unsigned y)
{
    // Tiny lookup table for the X offset of the Eight Queens
    // solution on row (Y % 8).
    return (0x24170635 >> ((y & 7) << 2)) & 7;
}   

inline int CameraSampler8Q::sampleX(unsigned index)
{
    return (blockX(index) << 3) + x8q(sampleY(index));
}

inline int CameraSampler8Q::blockX(unsigned index)
{
    return index % kBlocksWide;
}

inline unsigned CameraSampler8Q::blockIndex(unsigned index)
{
    return blockX(index) + blockY(index) * kBlocksWide;
}

inline int CameraSampler8Q::sampleY(unsigned index)
{
    return index / kBlocksWide;
}

inline int CameraSampler8Q::blockY(unsigned index)
{
    return sampleY(index) / kSamplesPerBlock;
}

inline bool CameraSampler8Q::next(unsigned &index, uint8_t &luminance)
{
    // Low four bits of the byteOffset we want (X coord + luminance byte)
    unsigned mask = x8q(y) * 2 + 1;

    while (iter.byteCount) {

        if ((iter.byteOffset & 0xF) == mask) {
            index = yIndex + (iter.byteOffset >> 4);
            luminance = *iter.data;

            iter.byteOffset++;
            iter.byteCount--;
            iter.data++;

            // Ready for more
            return true;
        }

        iter.byteOffset++;
        iter.byteCount--;
        iter.data++;
    }

    // Out of data
    return false;
}


inline CameraSamplerSobel::CameraSamplerSobel()
{
    memset(sobelX, 0, sizeof sobelX);
    memset(sobelY, 0, sizeof sobelY);
    memset(luminance, 0, sizeof luminance);
    memset(sobelXY, 0, sizeof sobelXY);
    memset(motion, 0, sizeof motion);
}

inline void CameraSamplerSobel::process(const Camera::VideoChunk &chunk)
{
    Camera::VideoChunk iter = chunk;
    int y = iter.line * Camera::kFields + iter.field;
    int yIndex = y * Camera::kPixelsPerLine;
    int yIndex8q = y * CameraSampler8Q::kBlocksWide;
    unsigned mask = CameraSampler8Q::x8q(y) * 2 + 1;

    while (iter.byteCount) {
        if (iter.byteOffset & 1) {
            // Luminance

            // Delta detect
            int index = yIndex + (iter.byteOffset >> 1);
            int prevL = luminance[index];
            int nextL = *iter.data;
            luminance[index] = nextL;
            int delta = nextL - prevL;
            int delta2 = delta * 2;

            // Sobel operators. No bounds-checking; instead we just pad the buffers

            *(sobelX + index -1 - Camera::kPixelsPerLine) -= delta;
            *(sobelX + index +1 - Camera::kPixelsPerLine) += delta;
            *(sobelX + index -1                         ) -= delta2;
            *(sobelX + index +1                         ) += delta2;
            *(sobelX + index -1 + Camera::kPixelsPerLine) -= delta;
            *(sobelX + index +1 + Camera::kPixelsPerLine) += delta;
            *(sobelY + index -1 - Camera::kPixelsPerLine) += delta;
            *(sobelY + index    - Camera::kPixelsPerLine) += delta2;
            *(sobelY + index +1 - Camera::kPixelsPerLine) += delta;
            *(sobelY + index -1 + Camera::kPixelsPerLine) -= delta;
            *(sobelY + index    + Camera::kPixelsPerLine) -= delta2;
            *(sobelY + index +1 + Camera::kPixelsPerLine) -= delta;       

            // Is this on our 8Q grid?
            if ((iter.byteOffset & 0xF) == mask) {
                unsigned i8q = yIndex8q + (iter.byteOffset >> 4);

                int sx = sobelX[index];
                int sy = sobelY[index];
                float sxy = sobelXY[i8q];
                int target = sx*sx + sy*sy;

                float motionSample = target - sxy;
                sxy = std::max<float>(1e-10, sxy + motionSample * kMotionFilterGain);
                sobelXY[i8q] = sxy;

                // Motion, adjusted for the novelty factor and brightness

                motionSample /= sxy;
                motionSample *= motionSample;
                motionSample *= nextL;
                motion[i8q] = motionSample;
            }
        }

        iter.byteOffset++;
        iter.byteCount--;
        iter.data++;
    }
}


inline CameraLuminanceBuffer::CameraLuminanceBuffer()
{
    memset(buffer, 0, sizeof buffer);
}

inline void CameraLuminanceBuffer::process(const Camera::VideoChunk &chunk)
{
    unsigned sampleIndex;
    uint8_t luminance;
 
    CameraSampler8Q s8q(chunk);
 
    while (s8q.next(sampleIndex, luminance)) {
        buffer[sampleIndex] = luminance;
    }
}
