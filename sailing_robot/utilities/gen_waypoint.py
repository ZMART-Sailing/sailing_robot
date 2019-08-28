import LatLon

left_top_point = LatLon.LatLon(29.867690, 121.538516)
left_down_point = LatLon.LatLon(29.867194, 121.538264)
right_top_point = LatLon.LatLon(29.867497, 121.539127)
right_down_point = LatLon.LatLon(29.867029, 121.538858)

start_point = left_top_point
start_line = 0

heading_v = start_point.heading_initial(left_down_point)
heading_h = start_point.heading_initial(right_top_point)

heading_sail = heading_h
heading_change = heading_v

lenght = 2.5
lenght /= 1000.0

gird_center = start_point.offset(heading_change, lenght).offset(heading_sail, lenght)

waypoint_list = []

waypoint_list.append(gird_center)

gird_center = gird_center.offset(heading_change, lenght * (start_line - 1))
if start_line != 1:
    waypoint_list.append(gird_center)

for i in range((21 - start_line) / 2):
    for i in range(3):
        gird_center = gird_center.offset(heading_sail, lenght * 6)
        waypoint_list.append(gird_center)
    gird_center = gird_center.offset(heading_change, lenght)
    waypoint_list.append(gird_center)
    for i in range(3):
        gird_center = gird_center.offset(heading_sail + 180, lenght * 6)
        waypoint_list.append(gird_center)
    gird_center = gird_center.offset(heading_change, lenght)
    waypoint_list.append(gird_center)
print('''# CAUTION:
# Some of these waypoints may not be suitable in all tide conditions!!
# for simplified entry of path, simply add list of waypoint names
# starts with list entry 0, continues until end is reached
# wp/list: ["1", "2", "3"]
# wp/list: ["1", "2","3","4"]
# triangle: 4, 5, 6
# position keeping: 4
# position keeping: 4, 5
# downwind: 6, 7''')
print('wp/tasks: [')
for i in range(len(waypoint_list)):
    print('{{"kind": "to_waypoint", "waypoint": "{0}", "target_radius": 2.5, "tack_voting_radius": 5}},'.format(i + 1))
print(''']
# distance within waypoint is accepted, in [m]
wp/target_radius: 4
wp/tack_voting_radius: 5


# Dictionary for looking up waypoints, using [lat, lon] format of NavSatFix
#''')
print('wp/table: {')
for i in range(len(waypoint_list)):
    print(
        '  "{0}": [{1}, {2}],'.format(i + 1, waypoint_list[i].lat.decimal_degree, waypoint_list[i].lon.decimal_degree))
print('}')
