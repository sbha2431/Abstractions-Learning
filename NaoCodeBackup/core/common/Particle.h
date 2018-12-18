#include <math/Pose2D.h>
#pragma once

struct Particle {
  float
    x, // X coordinate
    y, // Y coordinate
    t, // Theta (Orientation)
    w; // Weight
   Pose2D pos = Pose2D(t,x,y);
};
