"""Code for staying near a target point (2016 station keeping challenge)"""
from LatLon import LatLon
from shapely.geometry import Point
import time
import json

import taskbase


def angleAbsDistance(a, b):
    """Magnitude of the difference between two angles.

    Result should always be between 0 and 180.
    """
    distanceA = abs((a - b) % 360)
    distanceB = abs((b - a) % 360)
    return min(distanceA, distanceB)


class StationKeeping(taskbase.TaskBase):
    def __init__(self, nav, marker, radius = 5, accept_radius = 15, linger = 300, threshold = 10, marker_id = None,
                 kind = 'keep_station', name = '', *args, **kwargs):
        """Machinery to stay near a given point.

        This is meant to be started when we're already close to the marker; we'll
        normally put it immediately after a to_waypoint task to go to the marker.

        nav : a Navigation object for common machinery.
        marker_ll : a (lat, lon) point marking where we'll try to stay close to.
        linger : time in seconds to stay there
        radius : distance in metres which we'll try to bounce around the marker
        wind_angle : the absolute wind angle to sail (in degrees) when inside
           radius. This will automatically be flipped according to the tack.
        """
        self.nav = nav
        self.marker = marker
        self.marker_id = marker_id
        self.marker_xy = Point(self.nav.latlon_to_utm(self.marker.lat.decimal_degree, self.marker.lon.decimal_degree))
        self.linger = linger
        self.threshold = threshold
        self.radius = radius / 1000.0
        self.accept_radius = accept_radius
        self.start_time = None
        self.name = name
        self.debug_topics = [('dbg_keep_station_waypoint', 'String'), ]
        super(StationKeeping, self).__init__(**kwargs)
        self.last_wind_direction = None

    def calculate_waypoint_ll(self):
        return [self.marker]

    def calculate_waypoint(self, waypoints_ll = None):
        waypoints, waypoints_ll = super(StationKeeping, self).calculate_waypoint(waypoints_ll)
        self.taskdict['list'] = waypoints
        return waypoints, waypoints_ll

    def calculate_tasks(self):
        self.calculate_waypoint()

    def check_position(self):
        if self.nav.position_ll.distance(self.marker) <= self.accept_radius:
            self.nav.direct = True
            self.nav.task_direct_rudder_control = 40
            self.nav.task_direct_sailsheet_normalized = 0
            if self.start_time is None:
                self.start_time = time.time()
        else:
            self.nav.direct = False
            self.nav.task_direct_rudder_control = 0
            self.nav.task_direct_sailsheet_normalized = 0
            self.start_time = None

    def need_update(self):
        return False

    def check_end_condition(self):
        "Are we done yet?"
        return self.start_time is not None and time.time() - self.start_time > self.linger
