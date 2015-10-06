/*
 * Ecstatic Epiphany.
 *
 * Energy for inspiration.
 * Layers of self-organization.
 * Visual metaphors for creation.
 * Understanding, death, rebirth.
 * You are not the person you thought you'd become.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#include "lib/camera.h"
#include "narrator.h"

static Narrator narrator;

static void videoCallback(const Camera::VideoChunk &video, void *)
{
    narrator.flow.process(video);
}

int main(int argc, char **argv)
{
    narrator.runner.setLayout("layouts/window6x12.json");
    if (!narrator.runner.parseArguments(argc, argv)) {
        return 1;
    }

    narrator.setup();
    Camera::start(videoCallback);
    narrator.run();

    return 0;
}
