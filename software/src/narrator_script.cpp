/*
 * Tell a story with some lights.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#include "lib/sampler.h"
#include "narrator.h"
#include "chaos_particles.h"
#include "order_particles.h"
#include "rings.h"
#include "partner_dance.h"
#include "darkness.h"


int Narrator::script(int st, PRNG &prng)
{
    static ChaosParticles chaosA(runner.config["chaosParticles"]);
    static ChaosParticles chaosB(runner.config["chaosParticles"]);
    static OrderParticles orderParticles(runner.config["orderParticles"]);
    static RingsEffect ringsA(runner.config["ringsA"]);
    static RingsEffect ringsB(runner.config["ringsB"]);
    static RingsEffect ringsC(runner.config["ringsC"]);
    static PartnerDance partnerDance(runner.config["partnerDance"]);
    static DarknessEffect darkness;

    rapidjson::Value& config = runner.config["narrator"];
    Sampler s(prng.uniform32());

    switch (st) {

        default: {
            endCycle();
            return 10;
        }

        case 10: {
            ringsA.reseed(prng.uniform32());
            crossfade(&ringsA, s.value(config["ringsA-Crossfade"]));
            attention(s, config["ringsA-Attention"]);
            return 20;
        }

        case 20: {
            ringsB.reseed(prng.uniform32());
            crossfade(&ringsB, s.value(config["ringsB-Crossfade"]));
            attention(s, config["ringsB-Attention"]);
            return 30;
        }

        case 30: {
            // Biology happens, order emerges. Cellular look, emergent order.

            orderParticles.reseed(prng.uniform32());
            orderParticles.symmetry = 10;
            crossfade(&orderParticles, s.value(config["orderCrossfade"]));
            while (orderParticles.symmetry > 4) {
                attention(s, config["orderStepAttention"]);
                orderParticles.symmetry--;
            }
            attention(s, config["orderStepAttention"]);
            return 40;
        }

        case 40: {
            // Two partners, populations of particles.
            // Spiralling inwards. Depression. Beauty on the edge of destruction,
            // pressing forward until nothing remains.

            partnerDance.reseed(prng.uniform32());
            crossfade(&partnerDance, s.value(config["partnerCrossfade"]));
            attention(s, config["partnerAttention"]);
            return 50;
        }

        case 50: {
            // Sinking deeper. Interlude before a change.

            ringsC.reseed(prng.uniform32());
            crossfade(&ringsC, s.value(config["ringsC-Crossfade"]));
            attention(s, config["ringsC-Attention"]);
            return 60;
        }
    }
}
