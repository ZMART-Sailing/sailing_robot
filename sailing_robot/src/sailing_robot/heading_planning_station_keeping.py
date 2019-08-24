import heading_planning_laylines
import json


class HeadingPlan(heading_planning_laylines.HeadingPlan):
    def __init__(self, *args, **kwargs):
        super(HeadingPlan, self).__init__(*args, **kwargs)
        self.debug_topics.append(('dbg_ball_position', 'String'))

    def calculate_state_and_goal(self):
        if len(self.nav.relative_position_list) > 0:
            self.nav.calculate_ball_position()
            self.debug_pub('dbg_ball_position', json.dumps(
                [self.nav.ball_position.lat.decimal_degree, self.nav.ball_position.lon.decimal_degree]))
            self.update_waypoint(self.nav.ball_position)
        return super(HeadingPlan, self).calculate_state_and_goal()
