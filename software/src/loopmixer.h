/*
 * Dumb crossbar mixer for a bunch of looping tracks. For MultiDAC.
 */

/*
 * Copyright (c) 2015 Micah Elizabeth Scott <micah@misc.name>
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
#include <math.h>
#include "multidac.h"


inline float decibel(float db) {
    return powf(10.f, db / 10.f);
}


class LoopMixer
{
public:
    static const unsigned kNumChannels = MultiDAC::kNumChannels;
    static const unsigned kMaxTracks = 8;

    float master_gain;
    unsigned sample_number;
    unsigned loop_length;

    struct Track {
        float track_gain;
        float l_gains[kNumChannels];
        float r_gains[kNumChannels];
        int16_t *samples;
        unsigned sample_count;
    };

    Track tracks[kMaxTracks];

    LoopMixer();
    bool start(MultiDAC &multidac);

    // Warning, doesn't read the WAV header.
    // The file must be n a canonical format, with
    // stereo 16-bit little endian samples at 48 kHz
    bool load(unsigned index, const char *filename);

private:
    static void callback(MultiDAC::Frame *buffer, unsigned num_frames, void *userdata);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline LoopMixer::LoopMixer()
{
    master_gain = 0.f;
    sample_number = 0;
    loop_length = 0;
    memset(tracks, 0, sizeof tracks);
}

inline bool LoopMixer::start(MultiDAC &multidac)
{
    return multidac.start(callback, this);
}

inline bool LoopMixer::load(unsigned index, const char *filename)
{
    const unsigned kHeaderLen = 44;
    FILE *f = fopen(filename, "rb");
    if (!f) {
        perror(filename);
        return false;
    }

    fseek(f, 0, SEEK_END);
    unsigned data_bytes = ftell(f) - kHeaderLen;
    int16_t *new_samples = (int16_t*) malloc(data_bytes);
    rewind(f);
    fread(new_samples, 1, data_bytes, f);

    tracks[index].sample_count = 0;
    if (tracks[index].samples) {
        free(tracks[index].samples);
    }
    tracks[index].samples = new_samples;
    tracks[index].sample_count = data_bytes / 4;

    if (tracks[index].sample_count > loop_length) {
        loop_length = tracks[index].sample_count;
    }
    return true;
}

inline void LoopMixer::callback(MultiDAC::Frame *buffer, unsigned num_frames, void *userdata)
{
    LoopMixer *self = static_cast<LoopMixer*>(userdata);

    // Prep gains and buffer pointers outside the frame loop
    float gains[kMaxTracks];
    int16_t* buffers[kMaxTracks];
    for (unsigned track = 0; track < kMaxTracks; track++) {

        // Master scale factor that includes track gain, master gain, and conversion from 16-bit to 24-bit.
        // This gives us the smoothest volume control we can get with the hardware and samples we've got.
        float g = self->tracks[track].track_gain * self->master_gain * 256.f;

        buffers[track] = self->tracks[track].samples;
        if (buffers[track] == 0 || self->tracks[track].sample_count != self->loop_length) {
            // Don't support different lengths now. If anything's wrong, mute the track.
            g = 0;
        }
        gains[track] = g;
    }

    unsigned sample_number = self->sample_number;
    for (unsigned frame = 0; frame < num_frames; frame++) {
        if (sample_number >= self->loop_length) {
            sample_number = 0;
        }
        for (unsigned channel = 0; channel < kNumChannels; channel++) {
            int accum = 0;
            for (unsigned track = 0; track < kMaxTracks; track++) {
                float g = gains[track];
                if (g) {
                    int16_t *lr = buffers[track] + sample_number*2;
                    accum += (gains[track] * self->tracks[track].l_gains[channel]) * lr[0];
                    accum += (gains[track] * self->tracks[track].r_gains[channel]) * lr[1];
                }
            }
            buffer[frame].ch[channel] = accum;
        }
        sample_number++;
    }
    self->sample_number = sample_number;
}
