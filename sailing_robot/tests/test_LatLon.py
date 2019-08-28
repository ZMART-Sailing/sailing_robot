import LatLon
import json

mark = LatLon.LatLon(29.867080, 121.538874)
wp1 = LatLon.LatLon(29.867690, 121.538516)
wp2 = LatLon.LatLon(29.867194, 121.538264)
wp3 = LatLon.LatLon(29.867497, 121.539127)
wp4 = LatLon.LatLon(29.867029, 121.538858)

print wp2.heading_initial(wp1)
print wp2.heading_initial(wp4)

print mark.offset(mark)

waypoints = [LatLon.LatLon(*waypoint) for waypoint in
             [29.867475, 121.539166], [29.867390, 121.538790], [29.867544, 121.538868], [29.867631, 121.538592],]

for i in range(len(waypoints) - 1):
    print waypoints[i].distance(waypoints[i + 1]) * 1000
    print waypoints[i].heading_initial(waypoints[i + 1])
    print '------------'
print waypoints[-2].distance(waypoints[-1])
print waypoints[-2].heading_initial(waypoints[-1])

print waypoints
print type(waypoints[0])
print len(waypoints)

print waypoints[0].distance(waypoints[1])
print waypoints[0].heading_initial(waypoints[1])

print waypoints[0].offset(waypoints[0].heading_initial(waypoints[1]), 0.00581)
print waypoints[0].offset(waypoints[0].heading_initial(waypoints[1]) + 120, 0.005)
print waypoints[0].offset(waypoints[0].heading_initial(waypoints[1]) - 120, 0.005)

print waypoints[0].distance(waypoints[2])
print waypoints[0].heading_initial(waypoints[2])

print waypoints[2].distance(waypoints[1])
print waypoints[2].heading_initial(waypoints[1])

print '----------'
print dir(waypoint[0])
print waypoints[0]
print dir(waypoints[0].lat)
print dir(waypoints[0].lon)
json_str = json.dumps([waypoints[0].lat.decimal_degree, waypoints[0].lon.decimal_degree])
print json_str
json_obj = json.loads(json_str)
print json_obj
print type(json_obj)
