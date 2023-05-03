function [x, y] = gps_to_cartesian(goal_lat, goal_long)
[origin_lat, origin_long] = get_origin();

% Calculate the geodetic distance and azimuth between the two points
[distance, azimuth1, azimuth2] = geoddistance(origin_lat, origin_long, goal_lat, goal_long);

azimuth1 = deg2rad(azimuth1);
x = cos(azimuth1) * distance;
y = sin(azimuth1) * distance;
y = -y;
end

function [origin_lat, origin_long] = get_origin()
origin_lat = rosparam('get', 'origin_latitude');
origin_long = rosparam('get', 'origin_longitude');
end