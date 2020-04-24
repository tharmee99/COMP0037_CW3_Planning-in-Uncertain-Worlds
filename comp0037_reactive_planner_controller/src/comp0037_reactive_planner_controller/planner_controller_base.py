# This base class defines the API of a class which wraps around the planner and
# controller to produce reactive operations.
import rospy
from comp0037_reactive_planner_controller.aisle import Aisle

class PlannerControllerBase(object):

    def __init__(self, occupancyGrid, planner, controller):
        self.occupancyGrid = occupancyGrid
        self.planner = planner
        self.controller = controller

        # Rate parameter for the exponential distribution 
        self.waitLambda = 1
        # Cost of waiting one unit of time
        self.waitCost = 2
        # Maximum amount of time the robot is allowed to wait before replanning
        self.maxWaitTime = 30
        
        self.currentPlannedPath = None
        self.robotRadius = rospy.get_param('robot_radius', 0.2)

    def mapUpdateCallback(self, mapUpdateMessage):
        raise NotImplementedError()
    
    def driveToGoal(self, goal):
        raise NotImplementedError()
