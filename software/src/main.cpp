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
    mixer.tracks[0].track_gain = 1.;
    for (int n = 0; n < 9; n++) {
        mixer.tracks[0].l_gains[n] = 1.;
        mixer.tracks[0].r_gains[n] = 1.;
    }

    narrator.setup();
    mixer.start(multidac);
    sensor.init("data/eclsensor.rbf", "/dev/ttyAMA0");

    // while (1) {
    //     usleep(1);
    //     const EclSensor::Packet *p = sensor.poll();
    //     if (p) {
    //         if (p->tx_id == 0) {
    //             // Home cursor
    //             printf("\e[H");
    //         }
    //         printf("Tx %2d :", p->tx_id);
    //         for (unsigned i = 0; i < EclSensor::kRxCount; i++) {
    //             printf(" %5d", p->rx_timers[i]);
    //         }
    //         printf("\n");
    //     }
    // }

    narrator.run();

    return 0;
}
