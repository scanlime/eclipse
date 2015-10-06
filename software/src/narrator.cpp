/*
 * Infrastructure to tell a story with some lights.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#include <time.h>
#include "narrator.h"


Narrator::Narrator()
    : brightness(mixer)
{
    runner.setEffect(&brightness);
}

void Narrator::setup()
{
    flow.setConfig(runner.config["flow"]);
    brightness.set(0.0f, runner.config["brightnessLimit"].GetDouble());
    mixer.setConcurrency(runner.config["concurrency"].GetUint());
    runner.setMaxFrameRate(runner.config["fps"].GetDouble());
    currentState = runner.initialState;

    logFile = fopen(runner.config["narrator"]["logFile"].GetString(), "a");
    if (!logFile) {
        perror("Failed to open narrator log file");
    }
}    

void Narrator::run()
{
    PRNG prng;

    totalTime = 0;
    totalLoops = 0;
    prng.seed(time(0));

    while (true) {
        currentState = script(currentState, prng);
    }
}

void Narrator::endCycle()
{
    totalLoops++;

    if (logFile) {
        time_t now = time(NULL);
        char timeBuffer[256];
        ctime_r(&now, timeBuffer);        

        fprintf(logFile, "\n------ Summary : %s", timeBuffer);
        fprintf(logFile, "      loop total ");
        formatTime(logFile, totalTime); 
        fprintf(logFile, "  average ");
        formatTime(logFile, totalTime / totalLoops);
        fprintf(logFile, "\n");

        for (std::map<int, double>::iterator it = singleStateTime.begin(); it != singleStateTime.end(); it++) {
            fprintf(logFile, "state %-3d  total ", it->first);
            formatTime(logFile, it->second);
            fprintf(logFile, "  average ");
            formatTime(logFile, it->second / totalLoops);
            fprintf(logFile, "\n");
        } 

        fprintf(logFile, "Total loops: %d\n", totalLoops);
        fprintf(logFile, "----\n");
        fflush(logFile);
    }
}

EffectRunner::FrameStatus Narrator::doFrame()
{
    EffectRunner::FrameStatus st = runner.doFrame();

    totalTime += st.timeDelta;
    singleStateTime[currentState] += st.timeDelta;

    if (st.debugOutput && runner.isVerbose()) {
        fprintf(stderr, "\t[narrator] state = %d\n", currentState);
    }

    return st;
}

void Narrator::crossfade(Effect *to, float duration)
{
    int n = mixer.numChannels();
    if (n > 0) {
        mixer.add(to);
        for (float t = 0; t < duration; t += doFrame().timeDelta) {
            float q = t / duration;
            for (int i = 0; i < n; i++) {
                mixer.setFader(i, 1 - q);
            }
            mixer.setFader(n, q);
        }
    }
    mixer.set(to);
}

void Narrator::delayUntilDate(const rapidjson::Value& target)
{
    while (true) {
        double t = secondsAfterDate(target);
        if (t >= 0) {
            break;
        }

        EffectRunner::FrameStatus st = doFrame();
        if (st.debugOutput && runner.isVerbose()) {
            fprintf(stderr, "\t[delay] %f seconds left until date %s\n",
                -t, target.GetString());
        }
    }
}

void Narrator::delay(float seconds)
{
    while (seconds > 0) {
        EffectRunner::FrameStatus st = doFrame();
        seconds -= st.timeDelta;
        if (st.debugOutput && runner.isVerbose()) {
            fprintf(stderr, "\t[delay] %f seconds left\n", seconds);
        }
    }
}

void Narrator::delayForever()
{
    for (;;) {
        EffectRunner::FrameStatus st = doFrame();
        if (st.debugOutput && runner.isVerbose()) {
            fprintf(stderr, "\t[delay] forever\n");
        }
    }
}

void Narrator::formatTime(FILE *f, double s)
{
    fprintf(f, "%4d:%02d:%05.2f", (int)s / (60*60), ((int)s / 60) % 60, fmod(s, 60));
} 

double Narrator::secondsAfterDate(const rapidjson::Value& target)
{
    if (!target.IsString()) {
        fprintf(stderr, "Configured date is not a JSON String object\n");
        return 0;
    }

    time_t tNow = time(NULL);
    struct tm tmTarget;
    localtime_r(&tNow, &tmTarget);

    char *result = strptime(target.GetString(), "%Y-%m-%dT%H:%M:%S", &tmTarget);
    if (!result || *result) {
        fprintf(stderr, "Cannot parse \"%s\" as a date\n", target.GetString());
        return 0;
    }        

    time_t tTarget = mktime(&tmTarget);
    return difftime(tNow, tTarget);
}

Narrator::NEffectRunner::NEffectRunner()
    : initialState(0)
{
    if (!setConfig("data/config.json")) {
        fprintf(stderr, "Can't load default configuration file\n");
    }
}

bool Narrator::NEffectRunner::parseArgument(int &i, int &argc, char **argv)
{
    if (!strcmp(argv[i], "-state") && (i+1 < argc)) {
        initialState = atoi(argv[++i]);
        return true;
    }

    if (!strcmp(argv[i], "-config") && (i+1 < argc)) {
        if (!setConfig(argv[++i])) {
            fprintf(stderr, "Can't load config from %s\n", argv[i]);
            return false;
        }
        return true;
    }

    return EffectRunner::parseArgument(i, argc, argv);
}

void Narrator::NEffectRunner::argumentUsage()
{
    EffectRunner::argumentUsage();
    fprintf(stderr, " [-state ST] [-config FILE.json]");
}

bool Narrator::NEffectRunner::validateArguments()
{
    return config.IsObject() && EffectRunner::validateArguments();
}

bool Narrator::NEffectRunner::setConfig(const char *filename)
{
    FILE *f = fopen(filename, "r");
    if (!f) {
        return false;
    }

    rapidjson::FileStream istr(f);
    config.ParseStream<0>(istr);
    fclose(f);

    if (config.HasParseError()) {
        return false;
    }
    if (!config.IsObject()) {
        return false;
    }

    initialState = config["initialState"].GetInt();

    return true;
}

void Narrator::attention(Sampler &s, const rapidjson::Value& config)
{
    // Keep running until we run out of attention for the current scene.
    // Attention amounts and retention parameters come from JSON, and we show
    // our state during debug debug output.

    float bootstrap = s.value(config["bootstrap"]);
    float attention = s.value(config["initial"]);
    float brightnessDeltaMin = s.value(config["brightnessDeltaMin"]);
    float brightnessAverageMin = s.value(config["brightnessAverageMin"]);
    float rateBaseline = s.value(config["rateBaseline"]);
    float rateDark = s.value(config["rateDark"]);
    float rateStill = s.value(config["rateStill"]);

    delay(bootstrap);

    while (attention > 0) {
        EffectRunner::FrameStatus st = doFrame();

        float brightnessDelta = brightness.getTotalBrightnessDelta();
        float brightnessAverage = brightness.getAverageBrightness();

        float rate = rateBaseline
            + (brightnessDelta < brightnessDeltaMin ? rateStill : 0)
            + (brightnessAverage < brightnessAverageMin ? rateDark : 0);

        if (st.debugOutput && runner.isVerbose()) {
            fprintf(stderr, "\t[narrator] attention = %f\n", attention);
            fprintf(stderr, "\t[narrator] rate = %f\n", rate);
            fprintf(stderr, "\t[narrator] brightnessDelta = %f, threshold = %f %s\n",
                brightnessDelta, brightnessDeltaMin, brightnessDelta < brightnessDeltaMin ? "<<<" : "");
            fprintf(stderr, "\t[narrator] brightnessAverage = %f, threshold = %f %s\n",
                brightnessAverage, brightnessAverageMin, brightnessAverage < brightnessAverageMin ? "<<<" : "");
        }

        attention -= st.timeDelta * rate;
    }
}
