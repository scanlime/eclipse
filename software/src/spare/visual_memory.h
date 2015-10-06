/*
 * Experimental learning.
 * Likes motion, driven by randomness.
 * Estimates covariance, as a way to sense the environment.
 * Learns a relationship, LEDs to Camera and back.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <string>
#include <vector>
#include <bitset>
#include <sys/time.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include "lib/camera_sampler.h"
#include "lib/effect_runner.h"
#include "lib/jpge.h"
#include "lib/lodepng.h"
#include "lib/prng.h"
#include "latency_timer.h"


class VisualMemory
{
public:
    // Starts a dedicated processing thread
    void start(const char *memoryPath, const EffectRunner *runner, const EffectTap *tap);

    // Handle incoming video
    void process(const Camera::VideoChunk &chunk);

    // Snapshot memory state as a PNG file
    void debug(const char *outputPngFilename) const;

    // Read results, by LED pixel index. Between 0 and 1.
    float recall(unsigned ledIndex) const;

    // Camera feature extraction filters
    CameraLuminanceBuffer luminance;
    CameraSamplerSobel sobel;

    std::bitset<CameraSampler8Q::kSamples> learnFlags;
    std::bitset<CameraSampler8Q::kBlocks> recallFlags;

private:
    typedef double memory_t;
    typedef std::vector<memory_t> memoryVector_t;

    // Persistent mapped memory buffers updated on the learning thread
    memory_t *covariance;
    memory_t *sampleExpectedValue;
    memory_t *pixelExpectedValue;

    // Recall buffers, updated during learning
    memoryVector_t recallBuffer;
    memoryVector_t recallAccumulator;
    memoryVector_t recallTolerance;

    const EffectTap *tap;
    std::vector<unsigned> denseToSparsePixelIndex;

    // Separate learning thread
    tthread::thread *learnThread;
    static void learnThreadFunc(void *context);

    // Learning parameters
    static constexpr memory_t kMotionLearningThreshold = 3e-2;
    static constexpr memory_t kPermeability = 1e-5;
    static constexpr memory_t kExpectedValueGain = 1e-3;

    // Recall parameters
    static constexpr float kMotionRecallProportion = 0.30;
    static constexpr float kMotionRecallThresholdDecay = 1e-4;
    static constexpr float kMotionRecallThresholdLimit = 1e-4;
    static constexpr float kMotionThresholdMaxPeakRatio = 1e6;

    static constexpr memory_t kRecallFilterGain = 0.08;
    static constexpr memory_t kRecallToleranceGain = 0.001;

    // Main loop for learning thread
    void learnWorker();

    // Scalar sample utilities
    memory_t cameraSample(int sample);
    memory_t ledSample(int sparseIndex, const EffectTap::Frame *frame);
};


// Simple effect that visualizes recall data directly
class RecallDebugEffect : public Effect
{
public:
    RecallDebugEffect(VisualMemory *mem) : mem(mem) {}
    VisualMemory *mem;

    virtual void shader(Vec3& rgb, const PixelInfo &p) const {
        float f = mem->recall(p.index);
        rgb = Vec3(0,f,0);
    }
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline void VisualMemory::start(const char *memoryPath, const EffectRunner *runner, const EffectTap *tap)
{
    this->tap = tap;
    const Effect::PixelInfoVec &pixelInfo = runner->getPixelInfo();

    // Make a densely packed pixel index, skipping any unmapped pixels

    denseToSparsePixelIndex.clear();
    for (unsigned i = 0; i < pixelInfo.size(); ++i) {
        const Effect::PixelInfo &pixel = pixelInfo[i];
        if (pixel.isMapped()) {
            denseToSparsePixelIndex.push_back(i);
        }
    }

    // Calculate size of full visual memory

    unsigned denseSize = denseToSparsePixelIndex.size();
    unsigned cells = CameraSampler8Q::kSamples * denseSize;
    unsigned mappingSize = sizeof(memory_t) * (cells + denseSize + CameraSampler8Q::kSamples);
    int pagesize = getpagesize();
    mappingSize += pagesize - 1;
    mappingSize -= mappingSize % pagesize;

    // Recall and camera buffers

    recallBuffer.resize(pixelInfo.size());
    recallAccumulator.resize(denseSize);
    recallTolerance.resize(denseSize);

    std::fill(recallTolerance.begin(), recallTolerance.end(), 0);

    // Memory mapped file

    int fd = open(memoryPath, O_CREAT | O_RDWR | O_NOFOLLOW, 0666);
    if (fd < 0) {
        perror("vismem: Error opening mapping file");
        return;
    }

    if (ftruncate(fd, mappingSize)) {
        perror("vismem: Error setting length of mapping file");
        close(fd);
        return;
    }

    memory_t *mappedMemory = (memory_t*) mmap(0, mappingSize, PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, fd, 0);

    if (!mappedMemory) {
        perror("vismem: Error mapping memory file");
        close(fd);
        return;
    }

    covariance = mappedMemory;
    sampleExpectedValue = covariance + cells;
    pixelExpectedValue = sampleExpectedValue + CameraSampler8Q::kSamples;

    // Let the thread loose. This starts learning right away- no other thread should be
    // writing to the memory buffer from now on.

    learnThread = new tthread::thread(learnThreadFunc, this);
}

inline float VisualMemory::recall(unsigned ledIndex) const
{
    // Clamped and nonlinearly scaled
    float r = recallBuffer[ledIndex];
    r += 0.793;  // Cube root of 0.5
    r = std::max(0.0f, std::min(1.0f, r));
    return r*r*r;
}

inline void VisualMemory::process(const Camera::VideoChunk &chunk)
{
    luminance.process(chunk);
    sobel.process(chunk);
}

inline void VisualMemory::learnThreadFunc(void *context)
{
    VisualMemory *self = static_cast<VisualMemory*>(context);
    self->learnWorker();
}

inline VisualMemory::memory_t VisualMemory::cameraSample(int sample)
{
    int l = luminance.buffer[sample];
    return (l * l) / memory_t(255 * 255);
}

inline VisualMemory::memory_t VisualMemory::ledSample(int sparseIndex, const EffectTap::Frame *frame)
{
    const Vec3 &led = frame->colors[sparseIndex];

    Real r = std::min(1.0f, led[0]);
    Real g = std::min(1.0f, led[1]);
    Real b = std::min(1.0f, led[2]);

    return (r*r + g*g + b*b) / 3.0f;
}

inline void VisualMemory::learnWorker()
{
    unsigned denseSize = denseToSparsePixelIndex.size();

    // Fast inlined PRNG
    PRNG prng;
    prng.seed(84);

    // Coordinated motion sorting buffer
    std::vector< std::pair< float, unsigned > > cMotion;
    cMotion.resize(CameraSampler8Q::kBlocks);
    float cMotionThresholdPeak = 0.0f;

    // Performance counters
    unsigned loopCount = 0;
    struct timeval timeA, timeB;

    gettimeofday(&timeA, 0);
    gettimeofday(&timeB, 0);

    // Keep iterating over the memory buffer in the order it's stored
    while (true) {

        // For each cycle, keep an accumulator for the next recall buffer
        double recallTotal = 0;
        std::fill(recallAccumulator.begin(), recallAccumulator.end(), 0);

        /*
         * Update expected value filters
         */

        for (unsigned sampleIndex = 0; sampleIndex != CameraSampler8Q::kSamples; sampleIndex++) {
            memory_t v = cameraSample(sampleIndex);
            memory_t ev = sampleExpectedValue[sampleIndex];
            ev += (v * v - ev) * kExpectedValueGain;
            sampleExpectedValue[sampleIndex] = ev;
        }

        {
            const EffectTap::Frame *effectFrame = tap->get(LatencyTimer::kExpectedDelay);
            if (effectFrame) {
                for (unsigned denseIndex = 0; denseIndex != denseSize; denseIndex++) {
                    unsigned sparseIndex = denseToSparsePixelIndex[denseIndex];
                    memory_t v = ledSample(sparseIndex, effectFrame);
                    memory_t ev = pixelExpectedValue[denseIndex];
                    ev += (v * v - ev) * kExpectedValueGain;
                    pixelExpectedValue[denseIndex] = ev;
                }
            }
        }

        /*
         * Coordinated motion filter: sum of camera motion per-block, to detect clumps of motion
         * in a tight area for recall.
         */

        // 1. Clear coordinated motion accumulator, reset order
        for (unsigned blockIndex = 0; blockIndex != cMotion.size(); blockIndex++) {
            cMotion[blockIndex].first = 0;
            cMotion[blockIndex].second = blockIndex;
            recallFlags[blockIndex] = false;
        }

        // 2. Accumulate motion for each block
        for (unsigned sampleIndex = 0; sampleIndex != CameraSampler8Q::kSamples; sampleIndex++) {
            unsigned blockIndex = CameraSampler8Q::blockIndex(sampleIndex);
            cMotion[blockIndex].first += sq(sobel.motion[sampleIndex]);
        }

        // 3. Sort accumulators by total motion
        std::sort(cMotion.begin(), cMotion.end());

        // 4. Target the blocks within the top kMotionRecallProportion, smoothed with a leaky peak detector
        float cMotionThresholdTarget = cMotion[cMotion.size() * (1.0 - kMotionRecallProportion)].first;
        cMotionThresholdPeak = std::max( cMotionThresholdTarget,
                                std::min( kMotionThresholdMaxPeakRatio * cMotionThresholdTarget,
                                 cMotionThresholdPeak - cMotionThresholdPeak * kMotionRecallThresholdDecay));

        // printf("cMotion threshold filter=%e target=%e\n", cMotionThresholdPeak, cMotionThresholdTarget);

        // 5. Set recallFlags[] for use below and in the debug window
        for (unsigned i = 0; i < cMotion.size(); ++i) {
            recallFlags[cMotion[i].second] = cMotion[i].first >= cMotionThresholdPeak;
        }

        /*
         * Big loop, iterate over the huge covariance matrix. We update this matrix
         * sparsely, using a motion heuristic to avoid learning from areas of the image
         * that aren't moving.
         */

        for (unsigned sampleIndex = 0; sampleIndex != CameraSampler8Q::kSamples; sampleIndex++) {
            float motion = sobel.motion[sampleIndex];

            // Nonlinear probability distribution
            float r = prng.uniform();
            r *= r;
            r *= r;
            r *= r;

            // Increased motion increases the probability that we learn from this sample.
            bool isLearning = 1.0 / (kMotionLearningThreshold * (1 + motion)) < r;
            learnFlags[sampleIndex] = isLearning;
            if (!isLearning) {
                continue;
            }

            // Look up a delayed version of what the LEDs were doing then, to adjust for the system latency
            const EffectTap::Frame *effectFrame = tap->get(LatencyTimer::kExpectedDelay);
            if (!effectFrame) {
                // This frame isn't in our buffer yet
                usleep(10 * 1000);
                continue;
            }

            memory_t* cell = &covariance[sampleIndex * denseSize];
            memory_t cSample = cameraSample(sampleIndex) - sampleExpectedValue[sampleIndex];

            // Recall occurs when we exceed the coordinated motion threshold
            unsigned blockIndex = CameraSampler8Q::blockIndex(sampleIndex);
            bool isRecalling = recallFlags[blockIndex];

            // Learning and occurs on all LEDs for this sample
            for (unsigned denseIndex = 0; denseIndex != denseSize; denseIndex++, cell++) {
                unsigned sparseIndex = denseToSparsePixelIndex[denseIndex];
                memory_t state = *cell;

                // Compute covariance incrementally
                memory_t lSample = ledSample(sparseIndex, effectFrame);
                memory_t reinforcement = cSample * (lSample - pixelExpectedValue[denseIndex]);
                state = (state - state * kPermeability) + reinforcement;
                *cell = state;

                if (isRecalling) {
                    double acc = motion * motion * state;

                    // Integrate over all camera samples
                    recallAccumulator[sparseIndex] += acc;
                    recallTotal += acc;
                }
            }
        }

        /*
         * Update recallBuffer
         */

        double recallScale = recallTotal ? denseSize / recallTotal : 0.0;

        // printf("recallTotal %e recallScale %e\n", recallTotal, recallScale);

        for (unsigned denseIndex = 0; denseIndex != denseSize; denseIndex++) {
            unsigned sparseIndex = denseToSparsePixelIndex[denseIndex];

            memory_t tol = recallTolerance[denseIndex];
            memory_t target = recallAccumulator[denseIndex] * recallScale + tol;

            // Filtered update for recall buffer
            memory_t r = recallBuffer[sparseIndex];
            r += (target - r) * kRecallFilterGain;
            recallBuffer[sparseIndex] = r;

            if (recallTotal) {
                // Filtered update for tolerance
                recallTolerance[denseIndex] = tol - r * kRecallToleranceGain;
            }
        }

        /*
         * Periodic performance stats
         */
        
        loopCount++;
        gettimeofday(&timeB, 0);
        double timeDelta = (timeB.tv_sec - timeA.tv_sec) + 1e-6 * (timeB.tv_usec - timeA.tv_usec);
        if (timeDelta > 2.0f) {
            fprintf(stderr, "vismem: %.02f cycles / second\n", loopCount / timeDelta);
            loopCount = 0;
            timeA = timeB;
        }
    }
}

inline void VisualMemory::debug(const char *filename) const
{
    unsigned denseSize = denseToSparsePixelIndex.size();

    // Tiled array of camera samples, one per LED. Artificial square grid of LEDs.
    const int ledsWide = int(ceilf(sqrt(denseSize)));
    const int width = ledsWide * CameraSampler8Q::kBlocksWide;
    const int ledsHigh = (denseToSparsePixelIndex.size() + ledsWide - 1) / ledsWide;
    const int height = ledsHigh * CameraSampler8Q::kBlocksHigh;
    std::vector<uint8_t> image;
    image.resize(width * height * 3);

    // Maximum covariance
    memory_t cellMax = covariance[0];
    for (unsigned c = 1; c < (CameraSampler8Q::kSamples * denseSize); c++) {
        memory_t l = covariance[c];
        cellMax = std::max(cellMax, l);
    }

    fprintf(stderr, "vismem: range %f\n", cellMax);

    for (unsigned sample = 0; sample < CameraSampler8Q::kSamples; sample++) {
        for (unsigned led = 0; led < denseSize; led++) {

            int sx = CameraSampler8Q::blockX(sample);
            int sy = CameraSampler8Q::blockY(sample);

            int x = sx + (led % ledsWide) * CameraSampler8Q::kBlocksWide;
            int y = sy + (led / ledsWide) * CameraSampler8Q::kBlocksHigh;

            memory_t cell = covariance[ sample * denseSize + led ];
            uint8_t *pixel = &image[ 3 * (y * width + x) ];

            // Some cheesy HDR, so we can see more detail
            memory_t s = cell / cellMax;
            pixel[0] = std::min<memory_t>(255.5f, s*s*s*s * 255.0f + 0.5f);
            s *= 10;
            pixel[1] = std::min<memory_t>(255.5f, s*s*s*s * 255.0f + 0.5f);
            s *= 10;
            pixel[2] = std::min<memory_t>(255.5f, s*s*s*s * 255.0f + 0.5f);
        }
    }

    if (lodepng_encode_file(filename, &image[0], width, height, LCT_RGB, 8)) {
        fprintf(stderr, "vismem: error saving %s\n", filename);
    } else {
        fprintf(stderr, "vismem: saved %s\n", filename);
    }
}
