#ifndef KINEMATIC_CONSTANTS
#define KINEMATIC_CONSTANTS

#include <vector>

const int DOF = 6;

const double d0 = 0.4;
const double d1 = 0.025;
const double d2 = 0.455;
const double d3 = 0.035;
const double d4 = 0.42;
const double d5 = 0.08;

const std::vector<double> maxJointAngles = {2.96705972839, 0.78539816339, 2.72271363311, 3.22885911619, 2.09439510239, 6.10865238198};
const std::vector<double> minJointAngles = {-2.96705972839, -3.31612557879, -2.09439510239, -3.22885911619, -2.09439510239, -6.10865238198};

#endif