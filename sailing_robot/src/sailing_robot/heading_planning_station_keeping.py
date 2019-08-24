import heading_planning_laylines


class HeadingPlan(heading_planning_laylines.HeadingPlan):

    def calculate_state_and_goal(self):
        if len(self.nav.relative_position_list) > 0:
            self.update_waypoint(self.nav.calculate_ball_position())
        return super(HeadingPlan, self).calculate_state_and_goal()
