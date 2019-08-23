from collections import deque
import LatLon as ll
import math
from shapely.geometry import Point, Polygon
from sailing_robot import heading_planning_laylines


class HeadingPlan(heading_planning_laylines.HeadingPlan):

    def calculate_state_and_goal(self):
        return super(HeadingPlan, self).calculate_state_and_goal()
