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
#include "precursor.h"
#include "rings.h"
#include "partner_dance.h"
#include "forest.h"
#include "darkness.h"


int Narrator::script(int st, PRNG &prng)
{
    static ChaosParticles chaosA(flow, runner.config["chaosParticles"]);
    static ChaosParticles chaosB(flow, runner.config["chaosParticles"]);
    static OrderParticles orderParticles(flow, runner.config["orderParticles"]);
    static Precursor precursor(flow, runner.config["precursor"]);
    static RingsEffect ringsA(flow, runner.config["ringsA"]);
    static RingsEffect ringsB(flow, runner.config["ringsB"]);
    static RingsEffect ringsC(flow, runner.config["ringsC"]);
    static PartnerDance partnerDance(flow, runner.config["partnerDance"]);
    static CameraFlowDebugEffect flowDebugEffect(flow, runner.config["flowDebugEffect"]);
    static Forest forest(flow, runner.config["forest"]);
    static DarknessEffect darkness;

    rapidjson::Value& config = runner.config["narrator"];
    Sampler s(prng.uniform32());

    switch (st) {

        //////////////////////////////////////////////////////////////////////////////////////////////
        // Special states

        case 1: {
            // Darkness only ("off")
            crossfade(&darkness, 1);
            delayForever();
        }

        case 2: {
            // Precursor only (sleep mode)            
            precursor.reseed(prng.uniform32());
            crossfade(&precursor, 1);
            delayForever();
        }

        case 3: {
            // Debugging the computer vision system
            crossfade(&flowDebugEffect, 1);
            delayForever();
        }

        //////////////////////////////////////////////////////////////////////////////////////////////
        // Opening sequence

        case 0: {
            // Darkness until opening

            crossfade(&darkness, 1);
            delayUntilDate(config["opening"]["date"]);
            return config["opening"]["nextState"].GetInt();
        }

        //////////////////////////////////////////////////////////////////////////////////////////////
        // Cyclic states

        default: {
            endCycle();
            return 10;
        }

        case 10: {
            // Order trying to form out of the tiniest sparks; runs for an unpredictable time, fails.
            precursor.reseed(prng.uniform32());
            crossfade(&precursor, s.value(config["precursorCrossfade"]));

            // Bootstrap
            delay(s.value(config["precursorBootstrap"]));

            // Wait for darkness
            while (!precursor.isDone) {
                doFrame();
            }
            return 20;
        }

        case 20: {
            // Bang. Explosive energy, hints of self-organization

            ChaosParticles *pChaosA = &chaosA;
            ChaosParticles *pChaosB = &chaosB;
            
            int bangCount = s.value(config["bangCount"]);
            for (int i = 0; i < bangCount; i++) {
                pChaosA->reseed(prng.circularVector() * s.value(config["bangSeedRadius"]), prng.uniform32());
                crossfade(pChaosA, s.value(config["bangCrossfadeDuration"]));
                delay((1 << i) * s.value(config["bangDelayBasis"]));
                std::swap(pChaosA, pChaosB);
            }

            attention(s, config["bangAttention"]);

            return 30;
        }

        case 30: {
            // Textures of light, exploring something formless. Slow crossfade in
            ringsA.reseed(prng.uniform32());
            crossfade(&ringsA, s.value(config["ringsA-Crossfade"]));
            attention(s, config["ringsA-Attention"]);
            return 40;
        }

        case 40: {
            // Add energy, explore another layer.
            ringsB.reseed(prng.uniform32());
            crossfade(&ringsB, s.value(config["ringsB-Crossfade"]));
            attention(s, config["ringsB-Attention"]);
            return 50;
        }

        case 50: {
            // Biology happens, order emerges. Cellular look, emergent order.

            orderParticles.reseed(prng.uniform32());
            orderParticles.symmetry = 10;
            crossfade(&orderParticles, s.value(config["orderCrossfade"]));
            while (orderParticles.symmetry > 4) {
                attention(s, config["orderStepAttention"]);
                orderParticles.symmetry--;
            }
            attention(s, config["orderStepAttention"]);
            return 60;
        }

        case 60: {
            // Two partners, populations of particles.
            // Spiralling inwards. Depression. Beauty on the edge of destruction,
            // pressing forward until nothing remains.

            partnerDance.reseed(prng.uniform32());
            crossfade(&partnerDance, s.value(config["partnerCrossfade"]));
            attention(s, config["partnerAttention"]);
            return 70;
        }

        case 70: {
            // Sinking deeper. Interlude before a change.

            ringsC.reseed(prng.uniform32());
            crossfade(&ringsC, s.value(config["ringsC-Crossfade"]));
            attention(s, config["ringsC-Attention"]);
            return 80;
        }

        case 80: {
            // Continuous renewal and regrowth. Destruction happens unintentionally,
            // regrowth is quick and generative. The only way to lose is to stagnate.

            forest.reseed(prng.uniform32());
            crossfade(&forest, s.value(config["forestCrossfade"]));
            attention(s, config["forestAttention"]);
            return 90;
        }
    }
}
