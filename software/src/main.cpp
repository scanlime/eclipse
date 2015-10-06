#include "narrator.h"

static Narrator narrator;


int main(int argc, char **argv)
{
    narrator.runner.setLayout("layouts/strip64.json");
    if (!narrator.runner.parseArguments(argc, argv)) {
        return 1;
    }

    narrator.setup();
    narrator.run();

    return 0;
}
