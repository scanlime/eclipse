/*
 * Infrastructure to tell a story with some lights.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include "lib/rapidjson/rapidjson.h"
#include "lib/rapidjson/document.h"
#include "lib/effect.h"
#include "lib/effect_mixer.h"
#include "lib/effect_runner.h"
#include "lib/effect_tap.h"
#include "lib/prng.h"
#include "lib/sampler.h"
#include "lib/camera_flow.h"
#include "lib/brightness.h"


class Narrator
{
public:

    class NEffectRunner : public EffectRunner
    {   
    public:
        NEffectRunner();
        bool setConfig(const char *filename);

        int initialState;
        rapidjson::Document config;

    protected:
        virtual bool parseArgument(int &i, int &argc, char **argv);
        virtual void argumentUsage();
        virtual bool validateArguments();
    };

    Narrator();

    void setup();
    void run();

    CameraFlowAnalyzer flow;
    NEffectRunner runner;
    EffectMixer mixer;
    Brightness brightness;

private:
    int script(int st, PRNG &prng);
    EffectRunner::FrameStatus doFrame();
    void endCycle();

    void crossfade(Effect *to, float duration);
    void delay(float seconds);
    void delayForever();
    void attention(Sampler &s, const rapidjson::Value& config);
    void delayUntilDate(const rapidjson::Value& target);

    static double secondsAfterDate(const rapidjson::Value& target);
    static void formatTime(FILE *f, double s);

    FILE *logFile;
    unsigned totalLoops;
    double totalTime;
    std::map<int, double> singleStateTime;
    int currentState;
};
