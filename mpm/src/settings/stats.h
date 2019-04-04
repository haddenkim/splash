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
};