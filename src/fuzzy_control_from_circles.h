#ifndef FUZZY_CONTROL_FROM_CIRCLES_H
#define FUZZY_CONTROL_FROM_CIRCLES_H

#include"geometry_msgs/Twist.h"
#include<vector>
geometry_msgs::Twist fuzzy_control_from_circlrs(std::vector<Vec3f> RealCircles);

#endif // FUZZY_CONTROL_FROM_CIRCLES_H
