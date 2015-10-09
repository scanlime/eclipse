#include <math.h>
#include "narrator.h"
#include "eclsensor.h"
#include "multidac.h"
#include "loopmixer.h"

static Narrator narrator;
static EclSensor sensor;
static MultiDAC multidac;
static LoopMixer mixer;


int main(int argc, char **argv)
{
    narrator.runner.setLayout("layouts/grid32x16z.json");
    if (!narrator.runner.parseArguments(argc, argv)) {
        return 1;
    }

    mixer.master_gain = decibel(-25.);

    mixer.load(0, "data/progression Loop Drone.wav");
    mixer.load(1, "data/progression Air.wav");
    mixer.load(2, "data/progression Drums.wav");
    mixer.load(3, "data/progression Mini Strings.wav");
    mixer.load(4, "data/progression Glass.wav");
    mixer.load(5, "data/progression GlassWarp.wav");

    mixer.tracks[0].track_gain = 1.;
    for (int n = 0; n < 9; n++) {
        mixer.tracks[0].l_gains[n] = (n & 1);
        mixer.tracks[0].r_gains[n] = !(n & 1);
    }

    mixer.tracks[4].track_gain = decibel(-6);
    mixer.tracks[4].l_gains[3] = 1.f;

    mixer.tracks[2].track_gain = decibel(-15);
    for (int n = 0; n < 9; n++) {
        mixer.tracks[2].l_gains[n] = (n & 1);
        mixer.tracks[2].r_gains[n] = !(n & 1);
    }

    narrator.setup();
    mixer.start(multidac);
    sensor.init("data/eclsensor.rbf", "/dev/ttyAMA0");
    narrator.useSensor(sensor);
    narrator.run();

    return 0;
}
