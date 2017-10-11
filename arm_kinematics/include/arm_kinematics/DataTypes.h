#ifndef DATA_TYPES
#define DATA_TYPES

#include <matrix/math.hpp>

typedef matrix::Vector3<double> Vector3d;
typedef matrix::Vector<double, 6> JointValues;

struct Pose
{
    Vector3d position;
    Vector3d orientation;
};

#endif
