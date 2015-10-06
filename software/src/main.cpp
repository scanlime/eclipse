#include "narrator.h"
#include "eclsensor.h"

//static Narrator narrator;

static EclSensor sensor;


int main(int argc, char **argv)
{
	if (!sensor.init("data/eclsensor.rbf", "/dev/ttyAMA0")) {
		return 1;
	}

    // narrator.runner.setLayout("layouts/strip64.json");
    // if (!narrator.runner.parseArguments(argc, argv)) {
    //     return 1;
    // }

    // narrator.setup();
    // narrator.run();

    return 0;
}
