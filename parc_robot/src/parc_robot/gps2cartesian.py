#!/usr/bin/env python3

import rospy
import math
from geographiclib.geodesic import Geodesic

def get_origin():
  origin_lat, origin_long = rospy.get_param('origin_latitude'), rospy.get_param('origin_longitude')
  return origin_lat, origin_long

def gps_to_cartesian(goal_lat, goal_long):
  '''
  Finds the cartesian cordinate of the gps location with respect to the gps reference origin
  which is the same as Gazebo world origin.

  Args:
    goal_lat : Goal latitude
    goal_long : Goal longitude

  Returns:
    x : x cordinate in the sensor frame in meters
    y : y cordinate in the sensor frame in meters
  '''
  origin_lat, origin_long = get_origin()
  geod = Geodesic.WGS84  # define the WGS84 ellipsoid
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
  hypotenuse = distance = g['s12'] # access distance
  azimuth = g['azi1']

  # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
  # Convert azimuth to radians
  azimuth = math.radians(azimuth)
  x = adjacent = math.cos(azimuth) * hypotenuse
  y = opposite = math.sin(azimuth) * hypotenuse

  y = -y

  return x, y