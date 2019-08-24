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


class StationKeeping(taskbase.TaskRunnerBasedTaskBase):
    def __init__(self, nav, waypoint, radius = 5, accept_radius = 15, linger = 300, threshold = 10, waypoint_id = None,
                 kind = 'keep_station', name = '', *args, **kwargs):
        """Machinery to stay near a given point.

        This is meant to be started when we're already close to the waypoint; we'll
        normally put it immediately after a to_waypoint task to go to the waypoint.

        nav : a Navigation object for common machinery.
        waypoint_ll : a (lat, lon) point marking where we'll try to stay close to.
        linger : time in seconds to stay there
        radius : distance in metres which we'll try to bounce around the waypoint
        wind_angle : the absolute wind angle to sail (in degrees) when inside
           radius. This will automatically be flipped according to the tack.
        """
        self.nav = nav
        self.waypoint = waypoint
        self.waypoint_id = waypoint_id
        self.waypoint_xy = Point(
            self.nav.latlon_to_utm(self.waypoint.lat.decimal_degree, self.waypoint.lon.decimal_degree))
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
        self.last_wind_direction = self.nav.absolute_wind_direction()
        return [self.waypoint.offset(-self.last_wind_direction, self.radius),
                self.waypoint.offset(-self.last_wind_direction + 120, self.radius),
                self.waypoint.offset(-self.last_wind_direction - 120, self.radius)]

    def calculate_waypoint(self, waypoints_ll = None):
        waypoints, waypoints_ll = super(StationKeeping, self).calculate_waypoint(waypoints_ll)
        self.taskdict['list'] = waypoints
        return waypoints, waypoints_ll

    def calculate_tasks(self):
        self.taskdict['list'], _ = self.calculate_waypoint()

    def check_position(self):
        if self.start_time is None and self.nav.position_ll.distance(self.waypoint) < self.accept_radius:
            self.start_time = time.time()
        else:
            self.start_time = None

    def need_update(self):
        self.debug_pub('dbg_keep_station_waypoint', json.dumps(self.taskdict['list']))
        return self.last_wind_direction is None or angleAbsDistance(self.last_wind_direction,
                                                                    self.nav.absolute_wind_direction()) > self.threshold

    def update_tasks(self):
        _, waypoints = self.calculate_waypoint()
        for task, waypoint in zip(self.task_runner.tasks, waypoints):
            task.update_waypoint(waypoint)

    def check_end_condition(self):
        "Are we done yet?"
        return self.start_time is not None and time.time() - self.start_time > self.linger
