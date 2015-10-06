/*
 * Camera Flow - an object using sparse optical flow for
 * low-latency non-location-specific motion capture with a camera.
 *
 * This is based on sparse Lucas-Kanade optical flow estimation
 * combined with a simple form of foreground detection, allowing
 * efficient use of a small number of tracking points. This gives
 * us a rough but very fast optical flow estimate that we crunch
 * down into a fairly stable set of motion summary values for the
 * art to use.
 *
 * Motion vectors from multiple moving objects in the scene will
 * be summed. Coordinated motion from multiple objects will be
 * amplified, uncoordinated movement will become low-level noise.
 * This object will follow subtle movements from a single participant,
 * or try its best to account for the contributions of everyone
 * in a crowd.
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

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>
#include "effect.h"
#include "particle.h"
#include "color.h"
#include "camera.h"
#include "prng.h"


class CameraFlowCapture;

#ifdef __linux__
    // Currently use OpenCV VideoWriter on Linux only. It's broken on the Homebrew
    // OpenCV package for Mac, and I don't need it there anyway :(
    #define USE_OPENCV_VIDEO
#endif


class CameraFlowAnalyzer {
public:
    CameraFlowAnalyzer();

    // Process another chunk of video
    void process(const Camera::VideoChunk &chunk);

    // Change the transform we use to calculate model coordinates.
    void setTransform(Vec3 basisX, Vec3 basisY, Vec3 origin);

    // Set vision parameters from JSON object
    void setConfig(const rapidjson::Value &config);

    // Random number generator including entropy from video data
    PRNG prng;

private:
    friend class CameraFlowCapture;

    bool debug;                     // Super-verbose console and screenshot output
    unsigned debugFrameInterval;    // How often to output frames; 0 = never
    std::string debugVideoFile;     // Where to save encoded video debug output
    std::string debugVideoFourcc;   // FOURCC format code for debug video output
    std::string motionLogFile;      // Filename for motion integrator logs
    float debugMotionZoom;          // Scale factor for motion bars in debug video
    unsigned maxPoints;             // Max number of corner points to track at once
    unsigned decimate;              // Divide horizontal video resolution by skipping samples
    unsigned discoveryGridSpacing;  // Pixels per unit in the low-res point discovery sampling grid
    unsigned pointTrialPeriod;      // Number of frames to keep a point before discarding
    unsigned maxPointAge;           // Limit stale points; always discard after this age
    float minPointSpeed;            // Minimum speed in pixels/frame to keep a point
    float minEigThreshold;          // Minimum eigenvalue threshold to keep a point
    float deletePointProbability;   // Probability we'll delete a random tracking point to make room for a new one
    float motionFilterFast;         // First motion filter, higher frequency
    float motionFilterSlow;         // Second motion filter, low frequency
    float motionLogInterval;        // Seconds between motion integrator log records

    struct PointInfo {
        PointInfo();
        PointInfo(unsigned maxAge);

        float distanceTraveled;
        unsigned age;
        unsigned maxAge;
    };

    struct Field {
        cv::Mat frames[2];
        std::vector<cv::Point2f> points;
        std::vector<PointInfo> pointInfo;
    };

    #ifdef USE_OPENCV_VIDEO
        cv::VideoWriter debugVideoWriter;
    #endif
    Field fields[Camera::kFields];

    // Optical flow integrators, in 16:16 fixed point
    uint32_t integratorX, integratorY;

    // Total motion length integrator, in 16:16 fixed point
    uint32_t integratorL;

    // Length filtered at video rate
    uint32_t filterCaptureL;
    float filterSlowL;
    float filterFastL;

    // Current transform
    Vec3 basisX, basisY, origin;

    unsigned debugFrameCounter;
    uint32_t debugCaptureL;

    FILE *motionLogStream;
    double motionLogTimestamp;

    static uint32_t stringToFourCC(const std::string &f);
    void calculateFlow(Field &f);
    void clear();
    float instantaneousMotion() const;
};


class CameraFlowCapture {
public:
    CameraFlowCapture(const CameraFlowAnalyzer &analyzer);

    // Capture the current flow position, smoothly approach it
    void capture(float filterRate = 0.1f);

    // Set the last captured flow position to be the origin
    void origin();

    // Retrieve instantaneous, not integrated, value of the motion
    // per video field, filtered on the video thread. Not affected
    // by capture() / origin()
    float instantaneousMotion() const;

    // Raw x/y in pixels
    Vec2 pixels;

    // Model coordinates (arbitrary units)
    Vec3 model;

    // Total motion length since origin()
    float motionLength;

private:
    const CameraFlowAnalyzer &analyzer;
    uint32_t captureX, captureY, captureL;
    uint32_t originX, originY, originL;
};


class CameraFlowDebugEffect : public ParticleEffect
{
public:
    CameraFlowDebugEffect(CameraFlowAnalyzer& flow, const rapidjson::Value &config);

    virtual void beginFrame(const FrameInfo &f);
    virtual void debug(const DebugInfo &d);

private:
    float scale;
    float radius;
    float motionLengthScale;

    CameraFlowCapture flow;
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline CameraFlowAnalyzer::CameraFlowAnalyzer()
    : decimate(0), motionLogStream(NULL)
{
    prng.seed(29);

    // Default transform is identity
    setTransform( Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 0) );
}

inline void CameraFlowAnalyzer::setTransform(Vec3 basisX, Vec3 basisY, Vec3 origin)
{
    this->basisX = basisX;
    this->basisY = basisY;
    this->origin = origin;
}

inline void CameraFlowAnalyzer::setConfig(const rapidjson::Value &config)
{
    debug = config["debug"].GetBool();
    if (debug) {
        debugFrameInterval = config["debugFrameInterval"].GetUint();
        debugVideoFile = config["debugVideoFile"].GetString();
        debugVideoFourcc = config["debugVideoFourcc"].GetString();
    }

    maxPoints = config["maxPoints"].GetUint();
    decimate = config["decimate"].GetUint();
    discoveryGridSpacing = config["discoveryGridSpacing"].GetUint();
    pointTrialPeriod = config["pointTrialPeriod"].GetUint();
    maxPointAge = config["maxPointAge"].GetUint();
    minPointSpeed = config["minPointSpeed"].GetDouble();
    minEigThreshold = config["minEigThreshold"].GetDouble();
    deletePointProbability = config["deletePointProbability"].GetDouble();
    motionFilterFast = config["motionFilterFast"].GetDouble();
    motionFilterSlow = config["motionFilterSlow"].GetDouble();
    debugMotionZoom = config["debugMotionZoom"].GetDouble();
    motionLogFile = config["motionLogFile"].GetString();
    motionLogInterval = config["motionLogInterval"].GetDouble();

    const rapidjson::Value& t = config["transform"];
    if (t.IsArray() && t.Size() == 9) {
        setTransform( Vec3( t[0u].GetDouble(), t[1].GetDouble(), t[2].GetDouble() ),
                      Vec3( t[3 ].GetDouble(), t[4].GetDouble(), t[5].GetDouble() ),
                      Vec3( t[6 ].GetDouble(), t[7].GetDouble(), t[8].GetDouble() ));
    }

    // Resize arrays
    clear();
}

inline CameraFlowAnalyzer::PointInfo::PointInfo()
    : distanceTraveled(0), age(0), maxAge(0)
{}

inline CameraFlowAnalyzer::PointInfo::PointInfo(unsigned maxAge)
    : distanceTraveled(0), age(0), maxAge(maxAge)
{}

inline void CameraFlowAnalyzer::clear()
{
    integratorX = integratorY = integratorL = 0;
    debugFrameCounter = 0;
    debugCaptureL = 0;
    filterSlowL = 1.0f;
    filterFastL = 0;
    filterCaptureL = 0; 

    for (unsigned i = 0; i < Camera::kFields; i++) {
        if (decimate != 0) {
            for (unsigned j = 0; j < 2; j++) {
                fields[i].frames[j] = cv::Mat::zeros(Camera::kLinesPerField, Camera::kPixelsPerLine / decimate, CV_8UC1);
            }
        }
        fields[i].points.clear();
    }

    #ifdef USE_OPENCV_VIDEO
        if (debug && debugFrameInterval) {
            float fps = 60 / 1.001 / debugFrameInterval;
            debugVideoWriter.open(debugVideoFile.c_str(),
                stringToFourCC(debugVideoFourcc), fps, fields[0].frames[0].size());
        }
    #endif

    if (motionLogStream) {
        fclose(motionLogStream);
    }
    motionLogTimestamp = 0;
    motionLogStream = fopen(motionLogFile.c_str(), "a");
    if (!motionLogStream) {
        perror("Error opening motion log file");
    }
}

inline void CameraFlowAnalyzer::process(const Camera::VideoChunk &chunk)
{
    if (decimate == 0) {
        // Unconfigured
        return;
    }

    // Store decimated luminance values only

    Camera::VideoChunk iter = chunk;
    const uint8_t *limit = iter.data + chunk.byteCount;
    const unsigned bytesPerSample = decimate * 2;

    prng.remix(iter.byteOffset);
    prng.remix(iter.line);

    // Align to the next stored luminance value
    while ((iter.byteOffset % bytesPerSample) != 1) {
        iter.data++;
        iter.byteOffset++;
    }

    Field &f = fields[iter.field];
    cv::Mat &image = f.frames[1];
    uint8_t *dest = image.data +
        iter.line * (Camera::kPixelsPerLine / decimate)
        + iter.byteOffset / bytesPerSample;

    while (iter.data < limit) {
        *(dest++) = *iter.data;
        iter.data += bytesPerSample;
    }

    // End of field?

    if (iter.line == Camera::kLinesPerField - 1 &&
        chunk.byteCount + chunk.byteOffset == Camera::kBytesPerLine) {
        calculateFlow(f);

        // Check time elapsed for motion logging
        if (motionLogStream) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            double now = tv.tv_sec + tv.tv_usec * 1e-6;
            if (now >= motionLogTimestamp + motionLogInterval) {

                fprintf(motionLogStream, "%f 0x%x 0x%x 0x%x\n",
                    now, integratorX, integratorY, integratorL);
                fflush(motionLogStream);

                motionLogTimestamp = now;
            }
        }
    }
}

inline uint32_t CameraFlowAnalyzer::stringToFourCC(const std::string &f)
{
    return CV_FOURCC(f.size() >= 1 ? f[0] : 0,
                     f.size() >= 2 ? f[1] : 0,
                     f.size() >= 3 ? f[2] : 0,
                     f.size() >= 4 ? f[3] : 0);
}

inline float CameraFlowAnalyzer::instantaneousMotion() const
{
    float f = filterFastL;  // Filtered signal
    float s = filterSlowL;  // Approximate noise floor

    if (f < s) {
        return 0;
    }

    return log2f(f / (s + 1e-10));
}

inline void CameraFlowAnalyzer::calculateFlow(Field &f)
{            
    /*
     * Each NTSC field has its own independent flow calculator, so that we can react to
     * each field as soon as it arrives without worrying about correlating inter-field motion.
     */

    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    cv::Size subPixWinSize(6,6), winSize(15,15);

    int pointsToDelete = (int) prng.uniform(0, 1.0f + deletePointProbability);
    while (pointsToDelete > 0 && !f.points.empty()) {

        // Randomly delete a tracking point, to keep them from getting stuck in unhelpful
        // places and to help ensure there's a steady flow of slots for new points to spawn into.

        int i = std::min<int>(f.points.size()-1, prng.uniform(0, f.points.size()));

        if (debug) {
            fprintf(stderr, "flow[%d]: Random delete of point %d (age = %d, distance = %f)\n",
                (int)(&f - &fields[0]),
                i, f.pointInfo[i].age, f.pointInfo[i].distanceTraveled);
        }

        f.points.erase(f.points.begin() + i, f.points.begin() + i + 1);
        f.pointInfo.erase(f.pointInfo.begin() + i, f.pointInfo.begin() + i + 1);
        pointsToDelete--;
    }

    if (f.points.size() < maxPoints) {
        /*
         * Look for more points to track. We specifically want to focus on areas that are moving,
         * as if we were subtracting the background. (Actual background subtraction isn't
         * worth the CPU cost, in our case.) We also want to avoid points too near to any existing
         * ones.
         *
         * To quickly find some interesting points, I further decimate the image into a sparse
         * grid, and look for corners near the grid points that have the most motion.
         */

        cv::Size frameSize = f.frames[0].size();
        int gridWidth = frameSize.width / discoveryGridSpacing;
        int gridHeight = frameSize.height / discoveryGridSpacing;

        std::vector<bool> gridCoverage;
        gridCoverage.resize(gridWidth * gridHeight);
        std::fill(gridCoverage.begin(), gridCoverage.end(), false);

        // Calculate coverage of the discovery grid 
        for (unsigned i = 0; i < f.points.size(); i++) {
            int x = f.points[i].x / discoveryGridSpacing;
            int y = f.points[i].y / discoveryGridSpacing;
            unsigned idx = x + y * gridWidth;
            if (idx < gridCoverage.size()) {
                gridCoverage[idx] = true;
            }
        }

        // Look for the highest-motion point that isn't already on the grid, ignoring image edges.

        cv::Point2f bestPoint = cv::Point2f(0, 0);
        int bestDiff = 0;

        for (int y = 1; y < gridHeight - 1; y++) {
            for (int x = 1; x < gridWidth - 1; x++) {
                if (!gridCoverage[x + y * gridWidth]) {

                    // Random sampling bias, to avoid creating identical tracking points
                    const float s = discoveryGridSpacing * 0.4;
                    int pixX = x * discoveryGridSpacing + prng.uniform(-s, s);
                    int pixY = y * discoveryGridSpacing + prng.uniform(-s, s);

                    int diff = (int)f.frames[1].at<uint8_t>(pixY, pixX) - (int)f.frames[0].at<uint8_t>(pixY, pixX);
                    int diff2 = diff * diff;

                    if (diff2 > bestDiff) {
                        bestDiff = diff2;
                        bestPoint = cv::Point2f(pixX, pixY);
                    }
                }
            }
        }

        if (bestDiff > 0) {
            // Find a good corner near this point
            std::vector<cv::Point2f> newPoint;
            newPoint.push_back(bestPoint);
            cv::cornerSubPix(f.frames[0], newPoint, subPixWinSize, cv::Size(-1,-1), termcrit);

            // New point
            unsigned maxAge = prng.uniform(0, maxPointAge);
            f.points.push_back(newPoint[0]);
            f.pointInfo.push_back(PointInfo(maxAge));

            if (debug) {
                fprintf(stderr, "flow[%d]: Detecting new point (%f, %f) -> (%f, %f). %d points total\n",
                    (int)(&f - &fields[0]),
                    bestPoint.x, bestPoint.y,
                    newPoint[0].x, newPoint[0].y,
                    (int)f.points.size());
            }
        }
    }

    if (!f.points.empty()) {
        // Run Lucas-Kanade tracker

        std::vector<cv::Point2f> points;
        std::vector<uchar> status;
        std::vector<float> err;

        cv::Point2f numerator = cv::Point2f(0, 0);
        float denominator = 0;
        float numeratorL = 0, denominatorL = 0;

        cv::calcOpticalFlowPyrLK(f.frames[0], f.frames[1], f.points,
            points, status, err, winSize, 3, termcrit, 3, minEigThreshold);

        unsigned j = 0;
        for (unsigned i = 0; i < status.size(); i++) {

            if (status[i]) {
                // Point was found; update its info

                PointInfo info = f.pointInfo[i];
                cv::Point2f prevLocation = f.points[i];
                cv::Point2f nextLocation = points[i];
                cv::Point2f motion = nextLocation - prevLocation;
                float distance = sqrtf(motion.x * motion.x + motion.y * motion.y);

                info.age++;
                info.distanceTraveled += distance;

                // Is the point stale? Points that haven't moved will be discarded unless they're in
                // the initial trial period. Points that are too old will be discarded too, so stale
                // unreachable points don't get permanently included.

                if (info.age < pointTrialPeriod || 
                    (info.age < info.maxAge && info.distanceTraveled / info.age > minPointSpeed)) {

                    // Store point for next time
                    f.pointInfo[j] = info;
                    f.points[j] = nextLocation;
                    j++;

                    // Add to overall flow vector, using the point age and error to weight it
                    if (info.age > pointTrialPeriod) {
                        float weight = (info.age - pointTrialPeriod) / err[i];

                        numerator.x += motion.x * weight;
                        numerator.y += motion.y * weight;
                        denominator += weight;

                        numeratorL += distance * weight;
                        denominatorL += weight;
                    }

                } else if (debug) {
                    fprintf(stderr, "flow[%d]: Forgetting point %d with age=%d distance=%f speed=%f\n",
                        (int)(&f - &fields[0]),
                        i, info.age,
                        info.distanceTraveled, info.distanceTraveled / info.age);
                }
            } else if (debug) {
                fprintf(stderr, "flow[%d]: Point %d lost tracking\n",
                    (int)(&f - &fields[0]), i);
            }
        }

        f.points.resize(j);
        f.pointInfo.resize(j);

        // Update integrator using this field's data
        if (denominator) {
            integratorX += int32_t(numerator.x * 0x10000 / denominator);
            integratorY += int32_t(numerator.y * 0x10000 / denominator);
        }
        if (denominatorL) {
            integratorL += int32_t(numeratorL * 0x10000 / denominatorL);
        }

        if (debug) {
            fprintf(stderr, "flow[%d]: Tracking %d points, integrator (%08x, %08x) L=%08x denominator=%f\n",
                (int)(&f - &fields[0]),
                (int)f.points.size(),
                integratorX, integratorY, integratorL,
                denominator);
        }
    }

    // Update fixed-timestep motion filters on each field
    uint32_t cL = integratorL;
    float fL = int32_t(cL - filterCaptureL) * (1.0f / 0x10000);
    filterCaptureL = cL;
    filterSlowL += (fL - filterSlowL) * motionFilterSlow;
    filterFastL += (fL - filterFastL) * motionFilterFast;

    // Write frames to disk periodically in super-verbose debug mode
    #ifdef USE_OPENCV_VIDEO
        if (debug && debugFrameInterval) {
            debugFrameCounter++;
            if ((debugFrameCounter % debugFrameInterval) == 0 && debugVideoWriter.isOpened()) {

                cv::Mat frame;
                cv::cvtColor(f.frames[1], frame, cv::COLOR_GRAY2BGR);

                // Draw circles over each point; shade = age
                for (unsigned i = 0; i < f.points.size(); ++i) {
                    int l = std::min<int>(255, f.pointInfo[i].age);
                    cv::circle(frame, f.points[i], 2,
                            f.pointInfo[i].age < pointTrialPeriod
                                ? cv::Scalar(0, 0, 0)
                                : cv::Scalar(l, 64 + l/2, 255 - l),
                            1, CV_AA);
                }

                // Motion length since the last debug frame, scaled to units of pixels per field
                uint32_t prevL = debugCaptureL;
                uint32_t nextL = integratorL;
                float zoom = debugMotionZoom;
                debugCaptureL = nextL;
                float debugL = int32_t(nextL - prevL) * (zoom / 0x10000 / debugFrameInterval);
                cv::rectangle(frame,
                    cv::Point2f(1, 1),
                    cv::Point2f(3 + debugL, 4),
                    cv::Scalar(250, 176, 0), -1, CV_AA);

                // Filtered motion length dots
                cv::rectangle(frame,
                    cv::Point2f(1 + filterFastL * zoom, 6),
                    cv::Point2f(3 + filterFastL * zoom, 8),
                    cv::Scalar(255, 255, 190), -1, CV_AA);
                cv::rectangle(frame,
                    cv::Point2f(1 + filterSlowL * zoom, 10),
                    cv::Point2f(3 + filterSlowL * zoom, 12),
                    cv::Scalar(255, 190, 255), -1, CV_AA);

                // Computed instantaneoud motion
                float iM = instantaneousMotion();
                cv::rectangle(frame,
                    cv::Point2f(1, 14),
                    cv::Point2f(3 + iM * zoom, 16),
                    cv::Scalar(64, 64, 255), -1, CV_AA);

                debugVideoWriter.write(frame);
            }
        }
    #endif

    std::swap(f.frames[0], f.frames[1]);
}

inline CameraFlowCapture::CameraFlowCapture(const CameraFlowAnalyzer &analyzer)
    : analyzer(analyzer)
{
    originX = analyzer.integratorX;
    originY = analyzer.integratorY;
    originL = analyzer.integratorL;
    capture();
}       

inline void CameraFlowCapture::origin()
{
    originX = captureX;
    originY = captureY;
    originL = captureL;
}

inline float CameraFlowCapture::instantaneousMotion() const
{
    return analyzer.instantaneousMotion();
}

inline void CameraFlowCapture::capture(float filterRate)
{
    captureX = analyzer.integratorX;
    captureY = analyzer.integratorY;
    captureL = analyzer.integratorL;

    // Fixed point to floating point
    float targetX = (int32_t)(captureX - originX) / float(0x10000);
    float targetY = (int32_t)(captureY - originY) / float(0x10000);
    float targetMotionLength = (int32_t)(captureL - originL) / float(0x10000);

    // Smoothing filter
    pixels[0] += (targetX - pixels[0]) * filterRate;
    pixels[1] += (targetY - pixels[1]) * filterRate;
    motionLength += (targetMotionLength - motionLength) * filterRate;

    // Transform to model coordinates
    model = analyzer.origin + analyzer.basisX * pixels[0] + analyzer.basisY * pixels[1];
}

inline CameraFlowDebugEffect::CameraFlowDebugEffect(CameraFlowAnalyzer& flow, const rapidjson::Value &config)
    : scale(config["scale"].GetDouble()),
      radius(config["radius"].GetDouble()),
      motionLengthScale(config["motionLengthScale"].GetDouble()),
      flow(flow)
{}

inline void CameraFlowDebugEffect::beginFrame(const FrameInfo &f)
{
    flow.capture();

    appearance.resize(1);

    appearance[0].point = flow.model * scale;
    appearance[0].intensity = 1.0f;
    appearance[0].radius = radius + sqrtf(flow.instantaneousMotion()) * motionLengthScale;
    appearance[0].color = Vec3(1,1,1);

    for (unsigned i = 0; i < 3; i++) {
        if (appearance[0].point[i] < f.modelMin[i] || appearance[0].point[i] > f.modelMax[i]) {
            flow.origin();
        }
    }

    ParticleEffect::beginFrame(f);
}

inline void CameraFlowDebugEffect::debug(const DebugInfo &di)
{
    fprintf(stderr, "\t[flow] model = %f, %f, %f\n", flow.model[0], flow.model[1], flow.model[2]);
    fprintf(stderr, "\t[flow] motionLength = %f\n", flow.motionLength);
    fprintf(stderr, "\t[flow] instantaneousMotion = %f\n", flow.instantaneousMotion());
}


