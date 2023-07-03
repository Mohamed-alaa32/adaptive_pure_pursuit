"""
MODULE STRING
"""
import math
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import rospy
from nav_msgs.msg import Path, Odometry


# parameters
GAINLH = 0.3  # look forward gain rospy.get_param("/gain/lookahead", 0.3)
TARGETSPEED = 2  # rospy.get_param("/target_speed", 2)
LOOKAHEADCONSTANT = 2.0  # rospy.get_param("/lookahead", 2.5) # [m] look-ahead distance
# GAINTS = 0.2  # rospy.get_param("/gain/target_speed", 0.2)
MINSPEED = rospy.get_param("/speed/min", 2)
MAXSPEED = rospy.get_param("/speed/max", 10)
BASELENGTH = rospy.get_param(
    "/base_length", 2.5
)  # [m] difference between rear wheels and front wheels


@dataclass
class Position:
    """
    data class to store position of the vehicle
    """

    x: float
    y: float


class Car:
    """
    CLASS DOCSTRING
    """

    def __init__(
        self,
        position: Position = Position(0.0, 0.0),
        yaw: float = 0.0,
        currentSpeed: float = 0.0,
        lookahead: float = rospy.get_param("/lookahead", 1.5),
    ) -> None:
        """
        parameters
        ----------
        x : float
            x coordinate of the vehicle rear axle

        y : float
            y coordinate of the vehicle rear axle

        yaw : float
            yaw of the vehicle

        currentSpeed : float
            current speed of the vehicle

        rearX : float
            x coordinate of the rear of the vehicle

        rearY : float
            y coordinate of the rear of the vehicle
        """

        self.position: Position = Position(position.x, position.y)  # position in rear axle frame
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.lookAhead = lookahead
        self.rearX: float = self.position.x - ((BASELENGTH / 2) * math.cos(self.yaw))
        self.rearY: float = self.position.y - ((BASELENGTH / 2) * math.sin(self.yaw))
        self.poseList: List[Tuple[float, float]] = []

    def updateState(self, currentState: Odometry) -> None:
        """
        Update state of the car
        """

        self.position.x = currentState.pose.pose.position.x
        self.position.y = currentState.pose.pose.position.y
        self.yaw = currentState.pose.pose.orientation.z
        self.currentSpeed = currentState.twist.twist.linear.x
        self.poseList.append((self.position.x, self.position.y))
        self.lookAhead = self.currentSpeed * GAINLH + LOOKAHEADCONSTANT

    def calcDistance(self, pointX: float, pointY: float) -> float:
        """
        calculate the distance between the rear of the vehicle and a point

        Parameters
        ----------
        pointX : float
            x coordinate of the point

        pointY : float
            y coordinate of the point

        Returns
        -------
        distance : float
            distance between the rear of the vehicle and the point

        """
        distanceX: float = self.rearX - pointX
        distanceY: float = self.rearY - pointY
        distance: float = math.hypot(distanceX, distanceY)

        return distance

    def adaptivePurePursuitController(self, trajectory: Position) -> float:
        """
        DOCSTRING
        """
        trajX = trajectory.x
        trajY = trajectory.y

        alpha = math.atan2(trajY - self.rearY, trajX - self.rearX) - self.yaw

        delta = math.atan2(2.0 * BASELENGTH * math.sin(alpha) / self.lookAhead, 1.0)

        return delta

    def proportionalControl(self, delta: float) -> float:
        """
        DOCSTRING
        """

        targetSpeed: float = (20.0 / 3.6) / (abs(delta) * 4)
        targetSpeed = min(targetSpeed, MAXSPEED)
        targetSpeed = max(targetSpeed, MINSPEED)

        return targetSpeed


class WayPoints:
    """
    Class to store new waypoints to a list of waypoints and search for the suitable target point
    to follow with the pure pursuit algorithm
    """

    def __init__(self) -> None:
        """
        Parameters
        ----------
        xList : List[float]
            list of x coordinates of the waypoints

        yList : List[float]
            list of y coordinates of the waypoints

        oldNearestPointIndex : int
            index of the nearest point to the vehicle at the previous time step

        """
        self.waypoints = Path()
        self.xList: List[float] = []
        self.yList: List[float] = []
        self.point = Position(0.0, 0.0)
        self.oldNearestPointIndex: int = 0
        self.firstLoop: bool = False

    def updateWaypoints(self, waypointsMsg: Path) -> None:
        """
        callback function for recieving waypoints one time... All the points at once
        """
        # if self.xList == []:
        self.waypoints = waypointsMsg

    def targetPoints(self, ind: int, pind: int) -> Position:
        """
        DOCSTRING
        """
        if pind >= ind:
            ind = pind
        self.point.x = self.xList[ind]
        self.point.y = self.yList[ind]
        return self.point

    def searchTargetIndex(self, car: Car) -> int:
        """
        DOC STRING
        """

        if self.firstLoop is False:
            # search nearest point index
            print("Went here")
            for index, _ in enumerate(self.waypoints.poses):
                # Extracting and storing X and Y coordinates seperately in a list
                # to get minimum distance in first loop only
                # print(self.waypoints.poses[index])
                self.xList.append(self.waypoints.poses[index].pose.position.x)
                self.yList.append(self.waypoints.poses[index].pose.position.y)
            distanceX = [car.rearX - icx for icx in self.xList]
            distanceY = [car.rearY - icy for icy in self.yList]
            distance = np.hypot(distanceX, distanceY)
            # print(distance)
            if len(distance) != 0:
                ind: int = int(np.argmin(distance))
                self.firstLoop = True
                self.oldNearestPointIndex = ind
        lastIndex: int = len(self.xList) - 1
        ind = self.oldNearestPointIndex
        # print(self.xList)
        distanceThisIndex = car.calcDistance(self.xList[ind], self.yList[ind])

        while distanceThisIndex < car.lookAhead:

            if ind >= lastIndex - 1:
                ind = 0
            distanceNextIndex = car.calcDistance(self.xList[ind + 1], self.yList[ind + 1])

            ind = ind + 1
            distanceThisIndex = distanceNextIndex
        self.oldNearestPointIndex = ind

        return ind
