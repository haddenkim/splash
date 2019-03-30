#pragma once

struct RenderSettings
{
    RenderSettings()
    {
        m_showParticles = true;
        m_showGrid = true;
        m_showFloor = true;
    };

    // toggle visibility
    bool m_showParticles;
    bool m_showGrid;
    bool m_showFloor;
};