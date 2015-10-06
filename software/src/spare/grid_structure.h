/*
 * Categorizing lights by shape.
 * A sense of proprioception.
 * This one, rooted in math rather than observation.
 *
 * (c) 2014 Micah Elizabeth Scott
 * http://creativecommons.org/licenses/by/3.0/
 */

#pragma once

#include <vector>
#include <map>
#include <set>
#include "lib/effect.h"


struct GridStructure
{
    void init(const Effect::PixelInfoVec &pixels);

    typedef std::set<unsigned> PixelIndexSet;
    typedef std::pair<int, int> IntVec;
    typedef std::map<float, PixelIndexSet> CoordinateIndex;
    typedef std::map<IntVec, PixelIndexSet> GridIndex;

    CoordinateIndex coordIndex[3];
    GridIndex gridIndex;

    static IntVec intGridXY(const Effect::PixelInfo &pix);
};


/*****************************************************************************************
 *                                   Implementation
 *****************************************************************************************/


inline void GridStructure::init(const Effect::PixelInfoVec &pixels)
{
    for (unsigned j = 0; j < 3; j++) {
        coordIndex[j].clear();
    }
    gridIndex.clear();

    for (unsigned i = 0; i < pixels.size(); i++) {
        if (pixels[i].isMapped()) {
            // Componentwise index
            for (unsigned j = 0; j < 3; j++) {
                coordIndex[j][pixels[i].point[j]].insert(i);
            }

            gridIndex[intGridXY(pixels[i])].insert(i);
        }
    }
}

inline GridStructure::IntVec GridStructure::intGridXY(const Effect::PixelInfo &pix)
{
    Vec2 gridXY = pix.getVec2("gridXY");
    IntVec r(gridXY[0], gridXY[1]);
    return r;
}

