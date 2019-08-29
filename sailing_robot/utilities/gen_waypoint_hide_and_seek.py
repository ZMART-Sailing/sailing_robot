import LatLon

close_distance = 3
close_sail_len = 30
close = True

fir_start_point = LatLon.LatLon(29.867690, 121.538516)
fir_end_point = LatLon.LatLon(29.867194, 121.538264)
sec_start_point = LatLon.LatLon(29.867497, 121.539127)
sec_end_point = LatLon.LatLon(29.867029, 121.538858)

fir_len = fir_start_point.distance(fir_end_point)
offset = (fir_len - close_sail_len / 1000.0) / 2
fir_close_point = fir_start_point.offset(fir_start_point.heading_initial(fir_end_point), offset).offset(
    fir_start_point.heading_initial(sec_start_point) + (0 if close else 180), close_distance / 1000.0)
sec_close_point = fir_close_point.offset(sec_start_point.heading_initial(sec_end_point), close_sail_len / 1000.0)

waypoint_list = []

waypoint_list.append(fir_start_point)
waypoint_list.append(fir_end_point)
waypoint_list.append(fir_close_point)
waypoint_list.append(sec_close_point)

print('''# CAUTION:
# Some of these waypoints may not be suitable in all tide conditions!!
# for simplified entry of path, simply add list of waypoint names
# starts with list entry 0, continues until end is reached
# wp/list: ["1", "2", "3"]
# wp/list: ["1", "2","3","4"]
# triangle: 4, 5, 6
# position keeping: 4
# position keeping: 4, 5
# downwind: 6, 7


#wp/tasks: [
#{"kind": "to_waypoint", "waypoint": "1"},
#{"kind": "to_waypoint", "waypoint": "2"},
#]

wp/tasks: [
{"kind": "to_waypoint", "waypoint": "1"},
{"kind": "to_waypoint", "waypoint": "3"},
{"kind": "to_waypoint", "waypoint": "4"},
{"kind": "to_waypoint", "waypoint": "2"},
{"kind": "to_waypoint", "waypoint": "4"},
{"kind": "to_waypoint", "waypoint": "3"},
]

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
