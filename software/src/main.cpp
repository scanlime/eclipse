#include "narrator.h"
#include "eclsensor.h"
#include "multidac.h"

//static Narrator narrator;

static EclSensor sensor;
static MultiDAC multidac;


static void dac_callback(MultiDAC::Sample *buffer, unsigned num_samples, void *userdata)
{
    printf("cb!\n");
}


int main(int argc, char **argv)
{
    if (!multidac.start(dac_callback, 0)) {
        return 1;
    }

    return 0;

	if (!sensor.init("data/eclsensor.rbf", "/dev/ttyAMA0")) {
		return 1;
	}

    while (1) {
        usleep(1);
        const EclSensor::Packet *p = sensor.poll();
        if (p) {
            if (p->tx_id == 0) {
                // Home cursor
                printf("\e[H");
            }
            printf("Tx %2d :", p->tx_id);
            for (unsigned i = 0; i < EclSensor::kRxCount; i++) {
                printf(" %5d", p->rx_timers[i]);
            }
            printf("\n");
        }
    }

    // narrator.runner.setLayout("layouts/strip64.json");
    // if (!narrator.runner.parseArguments(argc, argv)) {
    //     return 1;
    // }

    // narrator.setup();
    // narrator.run();

    return 0;
}
