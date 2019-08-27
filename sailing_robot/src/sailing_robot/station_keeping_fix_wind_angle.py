"""Code for staying near a target point (2016 station keeping challenge)"""
from LatLon import LatLon
from shapely.geometry import Point
import time

import taskbase
import heading_planning_laylines_closely


class StationKeeping(taskbase.TaskBase):
    def __init__(self, nav, waypoint, linger = 300, radius = 5, wind_angle = 75, waypoint_id = None,
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
        self.radius = radius
        self.wind_angle = wind_angle
        self.goal_heading = 0
        self.sailing_state = 'normal'  # sailing state can be 'normal','switch_to_port_tack' or  'switch_to_stbd_tack'
        self.start_time = 0
        self.head_to_waypoint = heading_planning_laylines_closely.HeadingPlan(nav, self.waypoint, close_factor = 0.8,
                                                                              target_radius = radius,
                                                                              tack_voting_radius = radius)

        self.debug_topics = [('dbg_heading_to_waypoint', 'Float32'), ('dbg_distance_to_waypoint', 'Float32'),
                             ('dbg_goal_wind_angle', 'Float32'), ]
        self.debug_topics.extend(self.head_to_waypoint.debug_topics)

    def update_waypoint(self, waypoint, waypoint_id = None):
        self.waypoint = waypoint
        self.waypoint_id = waypoint_id
        self.waypoint_xy = Point(
            self.nav.latlon_to_utm(self.waypoint.lat.decimal_degree, self.waypoint.lon.decimal_degree))

    def init_ros(self):
        # Allow our sub-task to publish debugging stuff
        self.head_to_waypoint.debug_pub = self.debug_pub
        self.head_to_waypoint.log = self.log

    def start(self):
        self.start_time = time.time()

    def check_end_condition(self):
        "Are we done yet?"
        return time.time() - self.start_time > self.linger

    def calculate_state_and_goal(self):
        """Work out what we want the boat to do
        """
        dwp, hwp = self.nav.distance_and_heading(self.waypoint_xy)
        if dwp > self.radius:
            return self.head_to_waypoint.calculate_state_and_goal()

        self.debug_pub('dbg_distance_to_waypoint', dwp)
        self.debug_pub('dbg_heading_to_waypoint', hwp)

        if self.nav.angle_to_wind() < 0:
            goal_wind_angle = -self.wind_angle
        else:
            goal_wind_angle = self.wind_angle

        self.debug_pub('dbg_goal_wind_angle', goal_wind_angle)
        return 'normal', self.nav.wind_angle_to_heading(goal_wind_angle)
