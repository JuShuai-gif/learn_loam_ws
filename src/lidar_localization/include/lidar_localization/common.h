#ifndef _COMMON_H_
#define _COMMON_H_

#include <cmath>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;

inline double rad2deg(double randians)
{
    return randians * 180.0f / M_PI;
}

inline double deg2rad(double degrees){
    return degrees * M_PI / 180.0;
}


#endif // _COMMON_H_