# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot motion.

import rospy
import os
import time
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from aisle import Aisle
from search_grid import SearchGrid
from a_star_planner import AStarPlanner

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)
        
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        self.aisleToDriveDown = None

        self.exportDirectory = None
        self.initialPlanFlag = True

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):

        # Obtain occupancy grid and produce search grid from said occupancy grid
        self.gridUpdateLock.acquire()
        tempSearchGrid = SearchGrid.fromOccupancyGrid(self.occupancyGrid, self.robotRadius)
        self.gridUpdateLock.release()

        # Iterate through waypoints to see if the current path is still traverible.
        for waypointNumber in range(0,len(self.currentPlannedPath.waypoints)):
            coords = self.currentPlannedPath.waypoints[waypointNumber].coords

            # If any of the cells are marked as obstructed then stopDrivingToCurrentGoal() is called and false is returned
            if (tempSearchGrid.getCellFromCoords(coords).label == CellLabel.OBSTRUCTED):
                self.controller.stopDrivingToCurrentGoal()
                return False

        return True

    # Choose the first aisle the robot will initially drive down.
    # This is based on the prior.
    def chooseInitialAisle(self, startCellCoords, goalCellCoords):
        initialAisle = rospy.get_param('initial_aisle', None)
        if (initialAisle is not None):
            return Aisle[initialAisle]
        else:
            # List of available aisles
            aisles = [[Aisle.A, None],[Aisle.B, None],[Aisle.C, None],[Aisle.D, None],[Aisle.E, None]]

            # Iterating through aisles and computing the path cost via said aisle
            for aisle in aisles:
                aisle[1] = self.planPathToGoalViaAisle(startCellCoords,goalCellCoords,aisle[0], False).travelCost

            # Adding the average cost of waiting onto aisle B's path cost
            totalWaitCost = self.waitCost*(self.aisleBObstacleProbability/self.aisleBwaitLambda)
            aisleBPathCost = aisles[1][1]
            aisles[1][1] += totalWaitCost

            # Computing the threshold lambda value to choose C over B
            thresholdLambda = self.waitCost/(self.aisleBObstacleProbability*(aisles[2][1] - aisleBPathCost))

            # verbosity
            for aisle in aisles:
                rospy.loginfo("Cost of aisle {} policy: {}".format(aisle[0].name, aisle[1]))
            
            rospy.loginfo("Path cost via aisle B: {}".format(aisleBPathCost))
            rospy.loginfo("Cost of waiting in aisle B: {}".format(totalWaitCost))
            rospy.loginfo("Threshold lambda value: {}".format(thresholdLambda))

            # Identifying the aisle with the cheapers associated cost
            minCost = float('inf')
            bestAisle = Aisle.B
            for aisle in aisles:
                if (aisle[1] < minCost):
                    minCost = aisle[1]
                    bestAisle = aisle[0]

            rospy.loginfo("Choosing Aisle {} as the best aisle".format(bestAisle.name))

            return bestAisle

    # Choose the subdquent aisle the robot will drive down
    def chooseAisle(self, startCellCoords, goalCellCoords):
        return Aisle.C

    # Return whether the robot should wait for the obstacle to clear or not.
    def shouldWaitUntilTheObstacleClears(self, startCellCoords, goalCellCoords):

        # Computing cost of replanning
        nextChosenAisle = self.chooseAisle(startCellCoords,goalCellCoords)
        replanPathCost = self.planPathToGoalViaAisle(startCellCoords,goalCellCoords,nextChosenAisle, False).travelCost

        # Temporary planner object
        tempPlanner = AStarPlanner('Temporary A* Planner', self.occupancyGrid)
        tempPlanner.setPauseTime(0)
        tempPlanner.showGraphics = False
        # self.planner.windowHeightInPixels = rospy.get_param('maximum_window_height_in_pixels', 700)

        # Computing the cost of current cell to start cell
        tempPlanner.search(self.currentPlannedPath.waypoints[0].coords, startCellCoords)
        pathToStart = tempPlanner.extractPathToGoal()

        # Computing the cost of waiting, the expected value for the wait time is given as 1/lambda
        totalWaitCost =  self.waitCost*(1/self.aisleBwaitLambda)
        waitPathCost = self.currentPlannedPath.travelCost - pathToStart.travelCost
        waitPolicyCost = waitPathCost + totalWaitCost

        thresholdLambda = self.waitCost/(replanPathCost - waitPathCost)

        # Verbosity is always good
        rospy.loginfo("Cost of replanning policy is: {}".format(replanPathCost))
        rospy.loginfo("Cost of the waiting policy is: {}".format(waitPolicyCost))
        rospy.loginfo("Path-cost of the waiting path is: {}".format(waitPathCost))
        rospy.loginfo("Cost of waiting is: {}".format(totalWaitCost))
        rospy.loginfo("Threshold lambda value: {}".format(thresholdLambda))

        # If waiting cost is lower then wait else replan
        if(waitPolicyCost < replanPathCost):
            return True

        return False

    # This method will wait until the obstacle has cleared and the robot can move.
    def waitUntilTheObstacleClears(self, startCellCoords, goalCellCoords):
        
        # ROS time when the robot starts to wait
        startTime = rospy.get_time()

        # While the path is blocked keep waiting check occurs ever 0.5s
        while(not self.checkIfPathCurrentPathIsStillGood()):

            # If the robot has waitied longer than self.maxWaitTime, stop waiting
            if (rospy.get_time() - startTime >= self.maxWaitTime):
                rospy.logwarn("Waited too long... replanning")
                self.aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)
                break

            # Verbosity overdrive
            print("Waiting for obstacle to clear...")
            rospy.sleep(0.5)

        rospy.loginfo("Finished waiting...")
        
    # Aisle Coordinates are computed to use as waypoints to plan paths to
    def getAisleCellCoords(self, aisle):
        if (aisle == Aisle.A):
            return (6.25,6.375)
        elif (aisle == Aisle.B):
            return (10.5,6.375)
        elif (aisle == Aisle.C):
            return (14.75,6.375)
        elif (aisle == Aisle.D):
            return (18.5,6.375)
        elif (aisle == Aisle.E):
            return (22.5,6.375)


    # Plan a path to the goal which will go down the designated aisle. The code, as
    # currently implemented simply tries to drive from the start to the goal without
    # considering the aisle.
    def planPathToGoalViaAisle(self, startCellCoords, goalCellCoords, aisle, graphics = True):

        # Obtain aisle coords 
        aisleWorldCoords = self.getAisleCellCoords(aisle)
        aisleCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(aisleWorldCoords)

        # Show Graphics?
        self.planner.showGraphics = graphics

        # Plan path to aisle and extract the path
        pathToAisleFound = self.planner.search(startCellCoords, aisleCellCoords)
        if (pathToAisleFound is False):
            rospy.logwarn("Could not find a path to the goal at (%d, %d) via Aisle %s", \
                            goalCellCoords[0], goalCellCoords[1], aisle.name)
            return None
        initialSearchGrid = self.planner.searchGrid.getSearchGrid()
        pathToAisle = self.planner.extractPathToGoal()

        rospy.loginfo("Path Cost start To aisle: {}".format(pathToAisle.travelCost))

        # Plan path from aisle to goal and extract the path
        pathToGoalFound = self.planner.search(aisleCellCoords, goalCellCoords)    
        if (pathToGoalFound is False):
            rospy.logwarn("Could not find a path to the goal at (%d, %d) via Aisle %s", \
                            goalCellCoords[0], goalCellCoords[1], aisle.name)
            return None
        pathToGoal = self.planner.extractPathToGoal()

        rospy.loginfo("Path Cost aisle to goal: {}".format(pathToGoal.travelCost))

        # Concatenate the two paths to obtain the actual planned path via the chosen aisle
        pathToAisle.addToEnd(pathToGoal)

        if (graphics):

            # Merge the two search grids and update start/goal cells
            self.planner.searchGrid.leftMergeGrid(initialSearchGrid)
            self.planner.searchGridDrawer.setStartAndGoal(list(pathToAisle.waypoints)[0], list(pathToAisle.waypoints)[-1])
            self.planner.searchGridDrawer.update()

            # Plot the planned path on the search grid
            self.planner.searchGridDrawer.drawPathGraphics(pathToAisle)

            # Save the search grid as an image
            if ((self.initialPlanFlag) and (self.exportDirectory is not None)):
                saveFileName = os.path.join(self.exportDirectory, ("initial_search_grid_aisle" + str(aisle.name) + ".eps"))
                self.planner.searchGridDrawer.saveAsImage(saveFileName)
                self.initialPlanFlag = False

        return pathToAisle

    # This method drives the robot from the start to the final goal. It includes
    # choosing an aisle to drive down and both waiting and replanning behaviour.
    # Note that driving down an aisle is like introducing an intermediate waypoint.

    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        # Work out the initial aisle to drive down
        if (self.aisleToDriveDown is None):
            self.aisleToDriveDown = self.chooseInitialAisle(startCellCoords, goalCellCoords)

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.
        
        while rospy.is_shutdown() is False:

            # Plan a path from the robot's current position to the goal. This is called even
            # if the robot waited and used its existing path. This is more robust than, say,
            # stripping cells from the existing path.           
            
            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            self.currentPlannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, self.aisleToDriveDown)
            self.gridUpdateLock.release()

            # If we couldn't find a path, give up
            if self.currentPlannedPath is None:
                return False


            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

            # If we reached the goal, return
            if goalReached is True:
                return True

            # An obstacle blocked the robot's movement. Determine whether we need to
            # wait or replan.

            # Figure out where we are
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            # See if we should wait
            waitingGame = self.shouldWaitUntilTheObstacleClears(startCellCoords, goalCellCoords)

            # Depending upon the decision, either wait or determine the new aisle
            # we should drive down.
            if waitingGame is True:
                self.waitUntilTheObstacleClears(startCellCoords, goalCellCoords)
            else:
                self.aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        return False
            
            
