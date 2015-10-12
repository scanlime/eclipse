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
    narrator.runner.setLayout("layouts/front/layout.json");
    if (!narrator.runner.parseArguments(argc, argv)) {
        return 1;
    }

    narrator.setup();
    mixer.configure(narrator.runner.config["mixer"]);
    mixer.start(multidac);
    sensor.init("data/eclsensor.rbf", "/dev/ttyAMA0");
    narrator.useSensor(sensor);
    narrator.run();

    return 0;
}
