#pragma once

struct Stats{
    Stats()
    {
    };

    void reset()
    {
        stepCount = 0;
        simTime = 0.f;
    }

    int stepCount;
    float simTime;

    // sim clock
    int timeReset;
    int timeP2G;
    int timeGrid;
    int timeG2P;
    int timePart;
};