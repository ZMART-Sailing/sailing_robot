import LatLon as ll
import heading_planning_laylines
import json


class HeadingPlan(heading_planning_laylines.HeadingPlan):

    def __init__(self, nav, waypoint = ll.LatLon(50.742810, 1.014469),  # somewhere in the solent
                 target_radius = 2, offset_distance = 0, offset_heading = 0, *args, **kwargs):
        self.target = waypoint
        self.target_radius = target_radius
        self.offset_distance = offset_distance / 1000.0
        self.offset_heading = offset_heading
        self.nav = nav
        super(HeadingPlan, self).__init__(nav, self.calculate_real_waypoint(), target_radius, *args, **kwargs)
        self.debug_topics.append(('dbg_real_waypoint', 'String'))

    def calculate_real_waypoint(self):
        return self.target.offset(self.offset_heading, self.offset_distance)

    def calculate_state_and_goal(self):
        self.debug_pub('dbg_real_waypoint',
                       json.dumps([self.waypoint.lat.decimal_degree, self.waypoint.lon.decimal_degree]))
        return super(HeadingPlan, self).calculate_state_and_goal()
