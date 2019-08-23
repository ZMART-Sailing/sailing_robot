import LatLon
import json

waypoints = [LatLon.LatLon(*waypoint) for waypoint in
             [50.8896095, -1.383619], [50.8896095, -1.383701666667], [50.889478, -1.383536]]

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
