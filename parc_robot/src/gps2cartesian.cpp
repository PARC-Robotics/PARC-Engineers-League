#include "parc_robot/gps2cartesian.h"

GPS get_origin()
{
  GPS origin;
  ros::param::get("start_latitude", origin.latitude);

  double origin_long;
  ros::param::get("start_longitude", origin.longitude);

  return origin;
}

Cartesian gps_to_cartesian(double goal_lat, double goal_long)
{
  auto origin = get_origin();
  // Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
  const Geodesic& geod = Geodesic::WGS84();
  double distance, azimuth1, azimuth2;
  geod.Inverse(origin.latitude, origin.longitude, goal_lat, goal_long, distance, azimuth1, azimuth2);
  
  azimuth1 = azimuth1 * M_PI / 180.0;
  Cartesian cartesian;
  cartesian.x = std::cos(azimuth1) * distance;
  cartesian.y = std::cos(azimuth1) * distance;

  return cartesian;
}