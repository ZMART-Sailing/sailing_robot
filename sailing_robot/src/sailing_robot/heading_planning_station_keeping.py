import heading_planning_laylines
import json
import make_ros_tasks


class HeadingPlan(heading_planning_laylines.HeadingPlan):
    def __init__(self, *args, **kwargs):
        super(HeadingPlan, self).__init__(*args, **kwargs)
        self.debug_topics.extend([('dbg_ball_position', 'String'), ('dbg_keeping', 'Bool'), ])
        self.station_keeping = None
        self.taskdict = {
            'target_radius': self.target_radius,
            'tack_voting_radius': self.tack_voting_radius,
            'tasks': [{'kind': 'keep_station_obstacle'}, ]
        }

    def calculate_state_and_goal(self):
        self.debug_pub('dbg_keeping', self.station_keeping is not None)
        if self.station_keeping is None:
            if len(self.nav.relative_position_list) > 0:
                self.nav.calculate_ball_position()
                self.debug_pub('dbg_ball_position', json.dumps(
                    [self.nav.ball_position.lat.decimal_degree, self.nav.ball_position.lon.decimal_degree]))
                self.update_waypoint(self.nav.ball_position)
            dwp, hwp = self.nav.distance_and_heading(self.waypoint_xy)
            print dwp, hwp
            if dwp < self.target_radius:
                self.taskdict['tasks'][0].update({
                    'waypoint': (self.waypoint.lat.decimal_degree, self.waypoint.lon.decimal_degree)
                })
                print self.taskdict
                self.station_keeping = \
                    make_ros_tasks.make_ros_tasks(self.taskdict, self.nav, self.name + '/station_keeping')[0]
                print self.station_keeping

            return super(HeadingPlan, self).calculate_state_and_goal()
        else:
            return self.station_keeping.calculate_state_and_goal()

    def check_end_condition(self):
        """Are we there yet?"""
        return self.station_keeping is not None and self.station_keeping.check_end_condition()
