addpath('geographiclib-octave/inst');

rosinit

function [origin_lat, origin_long] = get_origin()
origin_lat = rosparam('get', 'start_latitude');
origin_long = rosparam('get', 'start_longitude');
end

function [x, y] = gps_to_cartesian(goal_lat, goal_long)
[origin_lat, origin_long] = get_origin()
[distance, azimuth] = gedistance(origin_lat, origin_long, goal_lat, goal_long)
azimuth = deg2rad(azimuth)
x = cos(azimuth) * distance
y = sin(azimuth) * distance
end