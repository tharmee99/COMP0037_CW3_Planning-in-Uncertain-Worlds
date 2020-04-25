# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot motion.

import rospy
import os
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
            return Aisle.B

    # Choose the subdquent aisle the robot will drive down
    def chooseAisle(self, startCellCoords, goalCellCoords):
        return Aisle.D

    # Return whether the robot should wait for the obstacle to clear or not.
    def shouldWaitUntilTheObstacleClears(self, startCellCoords, goalCellCoords):

        # Computing cost of replanning
        nextChosenAisle = self.chooseAisle(startCellCoords,goalCellCoords)
        replanPathCost = self.planPathToGoalViaAisle(startCellCoords,goalCellCoords,nextChosenAisle).travelCost

        # Computing the cost of waiting, the expected value for the wait time is given as 1/lambda
        waitPathCost = self.currentPlannedPath.travelCost + self.waitCost*(1/self.waitLambda)

        print("the cost of replanning is: {}".format(replanPathCost))
        print("the cost of waiting is: {}".format(waitPathCost))

        # If waiting cost is lower then wait else replan
        if(waitPathCost < replanPathCost):
            return True

        return False

    # This method will wait until the obstacle has cleared and the robot can move.
    def waitUntilTheObstacleClears(self):
        
        # ROS time when the robot starts to wait
        startTime = rospy.get_time()

        # While the path is blocked keep waiting check occurs ever 0.5s
        while(not self.checkIfPathCurrentPathIsStillGood()):

            # If the robot has waitied longer than self.maxWaitTime, stop waiting
            if (rospy.get_time() - startTime >= self.maxWaitTime):
                print("Waited too long... replanning")
                self.aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)
                break

            print("Waiting for obstacle to clear...")
            rospy.sleep(0.5)

        print("Finished waiting...")
        
    # Aisle Midpoints are computed to use as waypoints to plan paths to
    def getAisleMidpoint(self, aisle):
        if (aisle == Aisle.A):
            return (6.25,8.375)
        elif (aisle == Aisle.B):
            return (10.5,8.375)
        elif (aisle == Aisle.C):
            return (14.75,8.375)
        elif (aisle == Aisle.D):
            return (18.5,8.375)
        elif (aisle == Aisle.E):
            return (22.5,8.375)


    # Plan a path to the goal which will go down the designated aisle. The code, as
    # currently implemented simply tries to drive from the start to the goal without
    # considering the aisle.
    def planPathToGoalViaAisle(self, startCellCoords, goalCellCoords, aisle):

        # Obtain aisle coords 
        aisleWorldCoords = self.getAisleMidpoint(aisle)
        aisleCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(aisleWorldCoords)

        # Plan path to aisle and extract the path
        pathToAisleFound = self.planner.search(startCellCoords, aisleCellCoords)
        if (pathToAisleFound is False):
            rospy.logwarn("Could not find a path to the goal at (%d, %d) via Aisle %s", \
                            goalCellCoords[0], goalCellCoords[1], aisle.name)
            return None
        initialSearchGrid = self.planner.searchGrid.getSearchGrid()
        pathToAilse = self.planner.extractPathToGoal()

        # Plan path from aisle to goal and extract the path
        pathToGoalFound = self.planner.search(aisleCellCoords, goalCellCoords)    
        if (pathToGoalFound is False):
            rospy.logwarn("Could not find a path to the goal at (%d, %d) via Aisle %s", \
                            goalCellCoords[0], goalCellCoords[1], aisle.name)
            return None
        pathToGoal = self.planner.extractPathToGoal()

        # Concatenate the two paths to obtain the actual planned path via the chosen aisle
        pathToAilse.addToEnd(pathToGoal)

        # Merge the two search grids and update start/goal cells
        self.planner.searchGrid.leftMergeGrid(initialSearchGrid)
        self.planner.searchGridDrawer.setStartAndGoal(list(pathToAilse.waypoints)[0], list(pathToAilse.waypoints)[-1])
        self.planner.searchGridDrawer.update()

        # Plot the planned path on the search grid
        self.planner.searchGridDrawer.drawPathGraphics(pathToAilse)

        # Save the search grid as an image
        if ((self.initialPlanFlag) and (self.exportDirectory is not None)):
            saveFileName = os.path.join(self.exportDirectory, ("initial_search_grid_aisle" + str(aisle.name) + ".eps"))
            self.planner.searchGridDrawer.saveAsImage(saveFileName)
            self.initialPlanFlag = False

        return pathToAilse

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
                self.waitUntilTheObstacleClears()
            else:
                self.aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        return False
            
            
