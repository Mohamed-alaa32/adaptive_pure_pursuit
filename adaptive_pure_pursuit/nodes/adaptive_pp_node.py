"""
Initilization Pure Pursuit node for vehicle control
"""
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

from adaptive_pure_pursuit import Car, WayPoints, Position
# from adaptive_pure_pursuit.src.adaptive_pure_pursuit.adaptive_pure_pursuit import WayPoints, Car
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker


def main() -> None:
    """
    Main function for adaptive pure pursuit vehicle control node, subscribes to
    state and waypoints and publishes control actions
    """
    rospy.init_node("adaptive_pp_controller", anonymous=True)
    markerPub = rospy.Publisher("/TargetMarker", Marker, queue_size=10)
    controlActionPub = rospy.Publisher("/control_actions", AckermannDriveStamped, queue_size=10)
    waypoints = WayPoints()

    car = Car()
    marker = Marker()
    # rospy.Subscriber("/pathplanning/waypoints", Path, callback=waypoints.updateWaypoints)
    rospy.Subscriber("/random_waypoints", Path, callback=waypoints.updateWaypoints)
    rospy.Subscriber("/state", Odometry, callback=car.updateState)
    controlAction = AckermannDriveStamped()

    rate = rospy.Rate(rospy.get_param("/rate", 10))

    ind = 0
    # rospy.wait_for_message("/pathplanning/waypoints", Path)
    rospy.wait_for_message("/random_waypoints", Path)
    while not rospy.is_shutdown():
        pind = ind
        # print(ind)
        ind = waypoints.searchTargetIndex(car)
        targetPoints:Position = waypoints.targetPoints(ind, pind)
        delta = car.adaptivePurePursuitController(targetPoints)
        targetSpeed = car.proportionalControl(delta)

        controlAction.drive.steering_angle = delta
        controlAction.drive.speed = targetSpeed
        controlAction.drive.jerk = targetPoints.y
        controlAction.drive.acceleration = targetPoints.x
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = "map"
        marker.pose.position.x = waypoints.xList[ind]
        marker.pose.position.y = waypoints.yList[ind]
        marker.pose.position.z = -0.5
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.id = 0
        marker.header.stamp = rospy.Time.now()
        marker.ns = "state"
        markerPub.publish(marker)
        controlActionPub.publish(controlAction)



        rate.sleep()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass
