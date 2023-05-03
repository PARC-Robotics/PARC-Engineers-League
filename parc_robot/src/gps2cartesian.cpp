#include "parc_robot/gps2cartesian.h"
#include <GeographicLib/Geodesic.hpp>
#include <iostream>
#include <exception>
#include <ros/ros.h>

using namespace std;
using namespace GeographicLib;

GPS get_origin()
{
  GPS origin;
  ros::param::get("origin_latitude", origin.latitude);
  ros::param::get("origin_longitude", origin.longitude);

  return origin;
}

Cartesian gps_to_cartesian(double goal_lat, double goal_long)
{
  /*
  Finds the cartesian cordinate of the gps location with respect to the gps reference origin
  which is the same as Gazebo world origin.

  Args:
    goal_lat : Goal latitude
    goal_long : Goal longitude

  Returns:
    struct that contains x and y
      x : x cordinate in the sensor frame in meters
      y : y cordinate in the sensor frame in meters
    
  */
  auto origin = get_origin();
  
  const Geodesic& geod = Geodesic::WGS84();
  double distance, azimuth1, azimuth2;
  geod.Inverse(origin.latitude, origin.longitude, goal_lat, goal_long, distance, azimuth1, azimuth2);
  
  azimuth1 = azimuth1 * M_PI / 180.0;

  Cartesian cartesian;
  cartesian.x = std::cos(azimuth1) * distance;
  cartesian.y = std::sin(azimuth1) * distance;
  cartesian.y = -(cartesian.y);

  return cartesian;
}