#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

from rospy_message_converter import message_converter
from std_msgs.msg import String

from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

def pi():
    return math.pi

def deg2rad(x):
    return x * pi() / 180.

def rad2deg(x):
    return x * 180. / pi()


HOURSPERMINUTE = 1./60.
MINUTESPERSECOND = 1./60.
HOURSPERSECOND = HOURSPERMINUTE*MINUTESPERSECOND

FEETPERMILE= 5280.
METERSPERFOOT = .3048
FEETPERMETER = 1./METERSPERFOOT
MILESPERHOURTOMETERSPERSECOND = HOURSPERSECOND*FEETPERMILE*METERSPERFOOT;

def milesPerHourToMetersPerSecond(theMilesPerHour):
    return theMilesPerHour*MILESPERHOURTOMETERSPERSECOND;

def metersPerSecondToMilesPerHour(theMetersPerSecond):
    return theMetersPerSecond*1./MILESPERHOURTOMETERSPERSECOND;

def assertAlmostEqual(value_1, value_2, accuracy = 10**-8):
    assert(abs(value_1 - value_2) < accuracy), ("assertAlmostEqual-value_1:"+str(value_1)+", value_2:"+str(value_2))

def testMetersToMiles():
    # (25 m/sec)(3.2804 ft/m)(1/5280 mi/ft)(60 sec/min)(60 min/hr)=55.92 mi/hr
    assertAlmostEqual(55.92, metersPerSecondToMilesPerHour(25.),0.1)
    assertAlmostEqual(milesPerHourToMetersPerSecond(55.92), 25., 0.1)

def stampNewLane(theNewLane, theSequenceNumber, theOldLane):
    #rospy.loginfo("stampNewLane-theNewLane["+str(theSequenceNumber)+"]:\n"+str(theNewLane)
    #    +"\ntheSequenceNumber:"+str(theSequenceNumber))
    stampMessageWithHeader(theNewLane, theSequenceNumber, theOldLane.header.frame_id)

def stampWaypointTwist(theWaypoint, theSequenceNumber, theFrameId):
    #rospy.loginfo("stampWaypointTwist-theWaypoint["+str(theSequenceNumber)+"]:\n"+str(theWaypoint))
    stampMessageWithHeader(theWaypoint.twist, theSequenceNumber, theFrameId)

def stampMessageWithHeader(theMessageWithHeader, theSequenceNumber, theFrameId):
    #rospy.loginfo("stampMessageWithHeader-theMessageWithHeader["+str(theSequenceNumber)+"]:\n"+str(theMessageWithHeader))
    stampHeader(theMessageWithHeader.header, theSequenceNumber, theFrameId)

def stampHeader(theHeader, theSequenceNumber, theFrameId):
    # https://answers.ros.org/question/60209/what-is-the-proper-way-to-create-a-header-with-python/
    #srospy.loginfo("stampHeader-theHeader["+str(theSequenceNumber)+"]:\n"+str(theHeader))
    theHeader.stamp=rospy.Time.now()
    theHeader.seq=theSequenceNumber
    theHeader.frame_id=theFrameId

TESTINGWAYPOINTUPDATER=False
UNITTESTINGWAYPOINTUPDATER=False
DEBUGISWAYPOINTINFRONT=False
DEBUGPOSESTAMPEDYAW=False
LOGCREATEFINALWAYPOINTS=False

LOOKAHEAD_WPS = 200 if (not TESTINGWAYPOINTUPDATER) else 5 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def listSomePoseDistances(self, vehiclePoseStamped, lane, waypointRange):
        waypoints=lane.waypoints
        assert(len(waypoints)>0)
        rospy.loginfo("WaypointUpdater.listSomePoseDistances-vehiclePoseStamped:"+str(vehiclePoseStamped.header.frame_id)
            +", lane:"+str(lane.header.frame_id))
        # vehcile.pose & waypoint.pose are in the same coodinate system
        assert(vehiclePoseStamped.header.frame_id==lane.header.frame_id)
        for wp in range(waypointRange[0], waypointRange[1]):
            waypoint=waypoints[wp]
            vehiclePosition=vehiclePoseStamped.pose.position
            waypointPosition=waypoint.pose.pose.position
        #    rospy.loginfo("WaypointUpdater.listSomePoseDistances-type(vehiclePosition):"+str(type(vehiclePosition))+
        #        ", type(waypointPosition):"+str(type(waypointPosition)))
            vehiclePositionDictionary = message_converter.convert_ros_message_to_dictionary(vehiclePosition)
            waypointPositionDictionary = message_converter.convert_ros_message_to_dictionary(waypointPosition)
        #    rospy.loginfo("WaypointUpdater.listSomePoseDistances-vehiclePositionDictionary:"+str(vehiclePositionDictionary)+"\n->\n"
        #        +", waypointPositionDictionary:"+str(waypointPositionDictionary))
            vehicleYaw=self.poseStampedYaw(vehiclePoseStamped)
            waypointDistance=self.distanceToWaypoint(vehiclePosition, vehicleYaw, waypointPosition)
            rospy.loginfo("WaypointUpdater.listSomePoseDistances\nvehiclePositionDictionary"+str(vehiclePositionDictionary)+"\n->\n"
                +"waypointPositionDictionary "+str(wp)+":"+str(waypointPositionDictionary)+"\n="+str(waypointDistance)+"\n")
        #    rospy.signal_shutdown("listSomePoseDistances-exiting")

    def poseStampedYaw(self, poseStamped):
        # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        # https://www.youtube.com/watch?v=mFpH9KK7GvI&feature=youtu.be
        quaternion = (
            poseStamped.pose.orientation.x,
            poseStamped.pose.orientation.y,
            poseStamped.pose.orientation.z,
            poseStamped.pose.orientation.w
            )

        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        if (DEBUGPOSESTAMPEDYAW):
            rospy.loginfo("WaypointUpdater.vehicleYaw-quaternion:"+str(quaternion)
            +", euler:"+str(euler)
            +", roll:"+str(roll)+", pitch:"+str(pitch)+", yaw:"+str(yaw))
        return yaw

    def findWaypointInFront(self, vehiclePoseStamped):
        vehiclePosition=vehiclePoseStamped.pose.position
        vehicleYaw=self.poseStampedYaw(vehiclePoseStamped)
        if (DEBUGISWAYPOINTINFRONT):
            rospy.loginfo("WaypointUpdater.findWaypointInFront-vehicleYaw:"+str(vehicleYaw))
        return self.findFirstWaypointInFront(vehiclePosition, vehicleYaw, self.lane.waypoints)

    def findFirstWaypointInFront(self, vehiclePosition, vehicleYaw, waypoints):
        for wp in range(0, len(waypoints)):
            waypointPosition=waypoints[wp].pose.pose.position
            isInFront=self.isWaypointInFront(vehiclePosition, vehicleYaw, waypointPosition)
            if (DEBUGISWAYPOINTINFRONT):
                rospy.loginfo("WaypointUpdater.findFirstWaypointInFront-isInFront["+str(wp)+"]? "+str(isInFront))
            if (isInFront):
                return wp
        return -1

    def testFindFirstWaypointInFront(self):
        vehiclePosition=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Point", {"x":5.,"y":4.})
        # styx_msg/Waypoint
        waypoints=[]
        waypoint=message_converter.convert_dictionary_to_ros_message("styx_msgs/Waypoint", {"pose":{"pose":{"position":{"x":2., "y":2.}}}})
        waypoints.append(waypoint)
        waypoint=message_converter.convert_dictionary_to_ros_message("styx_msgs/Waypoint", {"pose":{"pose":{"position":{"x":2., "y":6.}}}})
        waypoints.append(waypoint)
        waypoint=message_converter.convert_dictionary_to_ros_message("styx_msgs/Waypoint", {"pose":{"pose":{"position":{"x":4.1, "y":5.}}}})
        waypoints.append(waypoint)
        waypoint=message_converter.convert_dictionary_to_ros_message("styx_msgs/Waypoint", {"pose":{"pose":{"position":{"x":6., "y":4.99}}}})
        waypoints.append(waypoint)
        waypoint=message_converter.convert_dictionary_to_ros_message("styx_msgs/Waypoint", {"pose":{"pose":{"position":{"x":6., "y":5.001}}}})
        waypoints.append(waypoint)
        rospy.loginfo("WaypointUpdater.testFindFirstWaypointInFront-waypoints:\n"+str(waypoints))
        vehicleYaw=deg2rad(90.)
        firstWaypoint=self.findFirstWaypointInFront(vehiclePosition, vehicleYaw, waypoints)
        assert(firstWaypoint==2)
        del waypoints[firstWaypoint]
        rospy.loginfo("WaypointUpdater.testFindFirstWaypointInFront-firstWaypoint:"+str(firstWaypoint)+", waypoints:\n"+str(waypoints))
        firstWaypoint=self.findFirstWaypointInFront(vehiclePosition, vehicleYaw, waypoints)
        assert(firstWaypoint==(len(waypoints)-1)), ("WaypointUpdater.testFindFirstWaypointInFront-firstWaypoint:"+str(firstWaypoint)+", len(waypoints):"+str(len(waypoints)))
        del waypoints[firstWaypoint]
        rospy.loginfo("WaypointUpdater.testFindFirstWaypointInFront--firstWaypoint:"+str(firstWaypoint)+", waypoints:\n"+str(waypoints))
        firstWaypoint=self.findFirstWaypointInFront(vehiclePosition, vehicleYaw, waypoints)
        assert(firstWaypoint==-1)

    INITIALTEMPORARYVELOCITY=milesPerHourToMetersPerSecond(10.);# 10 mph
    sequenceNumberForFinalWaypoints=0

    def createFinalWaypoints(self, firstWaypoint):
        waypoints=[]
        waypointSequenceNumber=0
        for wp in range(firstWaypoint, firstWaypoint+LOOKAHEAD_WPS):
            # do i need to have a circular range?
            # waypoint=styx_msgs.Waypoint
            currentVelocity=self.get_waypoint_velocity(self.lane.waypoints[wp])
            if (LOGCREATEFINALWAYPOINTS):
                rospy.loginfo("WaypointUpdater.createFinalWaypoints-currentVelocity["+str(wp)+"]:"+str(currentVelocity)
                    +"=="+str(metersPerSecondToMilesPerHour(currentVelocity))+" MPH")
            if (self.sequenceNumberForFinalWaypoints==0): # first call to create Lane
                currentVelocity=self.INITIALTEMPORARYVELOCITY
            else:
             if (currentVelocity>=self.INITIALTEMPORARYVELOCITY):
                currentVelocity+=(.1*currentVelocity)
             else:
                currentVelocity=self.INITIALTEMPORARYVELOCITY
            # set_waypoint_velocity(self, waypoints, waypoint, velocity)
            self.set_waypoint_velocity(self.lane.waypoints, wp, currentVelocity)
            waypoints.append(self.lane.waypoints[wp])
            # stampWaypoint(theWaypoint, theSequenceNumber, theFrameId):
            waypointSequenceNumber+=1
            stampWaypointTwist(waypoints[-1], waypointSequenceNumber, self.lane.header.frame_id)
        if (LOGCREATEFINALWAYPOINTS):
            rospy.loginfo("WaypointUpdater.createFinalWaypoints-waypoints:\n"+str(waypoints))
        finalWaypoints=Lane()
        finalWaypoints.waypoints=waypoints
        # https://answers.ros.org/question/60209/what-is-the-proper-way-to-create-a-header-with-python/
        # finalWaypoints.stamp=rospy.Time.now()
        # stampMessageWithHeader(theSequenceNumber, theFrameId, theMessageWithHeader):
        self.sequenceNumberForFinalWaypoints+=1
        stampNewLane(finalWaypoints, self.sequenceNumberForFinalWaypoints, self.lane)
        if (LOGCREATEFINALWAYPOINTS):
            rospy.loginfo("WaypointUpdater.createFinalWaypoints-finalWaypoints:\n"+str(finalWaypoints))
        return finalWaypoints

    previousFirstWaypoint=-1;
    firstWaypointRepeating=0

    def pose_cb(self, vehiclePoseStamped):
        # TODO: Implement
        #poseDictionary = message_converter.convert_ros_message_to_dictionary(vehiclePoseStamped)
        #srospy.loginfo("WaypointUpdater.pose_cb-poseDictionary:"+str(poseDictionary))

        if UNITTESTINGWAYPOINTUPDATER:
            self.listSomePoseDistances(vehiclePoseStamped, self.lane, [260,290])

        firstWaypoint=self.findWaypointInFront(vehiclePoseStamped)
        if (firstWaypoint!=self.previousFirstWaypoint):
            rospy.loginfo("WaypointUpdater.pose_cb-vehiclePoseStamped:\n"+str(vehiclePoseStamped))
            self.previousFirstWaypoint=firstWaypoint
            rospy.loginfo("WaypointUpdater.pose_cb-firstWaypoint:"+str(firstWaypoint)
                +", sequenceNumberForFinalWaypoints:"+str(self.sequenceNumberForFinalWaypoints))
            finalWaypoints=self.createFinalWaypoints(firstWaypoint)
            rospy.loginfo("WaypointUpdater.pose_cb-final_waypoints_pub.publish:\n"+str(finalWaypoints))
            self.final_waypoints_pub.publish(finalWaypoints)
            self.firstWaypointRepeating=0
        else:
            self.firstWaypointRepeating+=1
            if (self.firstWaypointRepeating%100==0):
                rospy.loginfo("WaypointUpdater.pose_cb-firstWaypointRepeating:"+str(self.firstWaypointRepeating)
                    +", unchanged firstWaypoint:"+str(firstWaypoint)
                    +", vehicle position:"+str(vehiclePoseStamped.pose.position))

        #rospy.signal_shutdown("pose_cb-exiting")
        pass

    INFRONTANGLE=deg2rad(45.)
    def isWaypointInFront(self, vehiclePosition, vehicleYaw, wayPointPosition):
        waypointTranslatedToOrigin=[wayPointPosition.x-vehiclePosition.x, wayPointPosition.y-vehiclePosition.y]
        waypointRotatededAtOrigin=[(waypointTranslatedToOrigin[0] * math.cos(-vehicleYaw) - waypointTranslatedToOrigin[1]*math.sin(-vehicleYaw)),
            (waypointTranslatedToOrigin[0] * math.sin(-vehicleYaw) + waypointTranslatedToOrigin[1]*math.cos(-vehicleYaw))]
        if (DEBUGISWAYPOINTINFRONT):
            rospy.loginfo("WaypointUpdater.isWaypointInFront-waypointTranslatedToOrigin:"+str(waypointTranslatedToOrigin)
            +", waypointRotatededAtOrigin:"+str(waypointRotatededAtOrigin))
        # waypoint/vehicle translated & rotated so that vehicle is at 0,0 and pointing in +x direction
        # if waypoint is in -x direction -> it is behind the vehicle
        if (waypointRotatededAtOrigin[0]<0) :
            return False
        # waypoint is in the +x,+y or +x,-y quadrant
        else:
            waypointAngle=math.atan2(waypointRotatededAtOrigin[1], waypointRotatededAtOrigin[0])
            if (DEBUGISWAYPOINTINFRONT):
                rospy.loginfo("WaypointUpdater.isWaypointInFront-waypointAngle:"+str(rad2deg(waypointAngle))+" degrees")
            # outside a INFRONTANGLE fan shadow in front of car?
            if (abs(waypointAngle)>self.INFRONTANGLE):
                return False
        return True 

    def isPoseInFront(self, vehiclePose, vehicleYaw, wayPointPose):
        return self.isWaypointInFront(vehiclePose.position, vehicleYaw, wayPointPose.position)

    def testIsPoseInFront(self):
        vehiclePose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":5.,"y":4.}})
        vehicleYaw=deg2rad(90.)
        # 2,2 -> -2,3
        wayPointPose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":2., "y":2.}})
        test1 = self.isPoseInFront(vehiclePose, vehicleYaw, wayPointPose)
        rospy.loginfo("WaypointUpdater.testisPoseInFront-test1:"+str(test1))
        assert(not test1)
        vehiclePose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":5.,"y":4.}})
        # 2,6 -> 2,3
        wayPointPose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":2., "y":6.}})
        # in -x space -> false
        test2 = self.isPoseInFront(vehiclePose, vehicleYaw, wayPointPose)
        rospy.loginfo("WaypointUpdater.testisPoseInFront-test2:"+str(test2))
        assert(not test2)
        vehiclePose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":5.,"y":4.}})
        # 4,5 -> 1,1
        wayPointPose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":4.1, "y":5.}})
        # in +x space, just inside the left hand side of the fan -> true
        test3 = self.isPoseInFront(vehiclePose, vehicleYaw, wayPointPose)
        rospy.loginfo("WaypointUpdater.testisPoseInFront-test3:"+str(test3))
        assert(test3) 
        vehiclePose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":5.,"y":4.}})
        # 6,5 -> 1,-1
        wayPointPose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":6., "y":4.99}})
        # in +x space, just outside the right hand side of the fan -> false
        test4 = self.isPoseInFront(vehiclePose, vehicleYaw, wayPointPose)
        rospy.loginfo("WaypointUpdater.testisPoseInFront-test4:"+str(test4))
        assert(not test4) 
        vehiclePose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":5.,"y":4.}})
        # 6,5 -> 1,-1
        wayPointPose=message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", {"position":{"x":6., "y":5.001}})
        # in +x space, just inside the right hand side of the fan -> true
        test5 = self.isPoseInFront(vehiclePose, vehicleYaw, wayPointPose)
        rospy.loginfo("WaypointUpdater.testIsInFront-test5:"+str(test5))
        assert(test5) 

    def listSomeDistancesBetweenWaypoints(self, waypoints, waypointRange):
        for wp in range(waypointRange[0], waypointRange[1]):
            waypointDistance=self.distance(waypoints, wp, wp+1)
            rospy.loginfo("WaypointUpdater.listSomeDistancesBetweenWaypoints-"+str(wp)+"->"+str(wp+1)+":"+str(waypointDistance))

    def listSomeWaypoints(self, waypoints):
        w=0
        for waypoint in waypoints:
            waypointDictionary = message_converter.convert_ros_message_to_dictionary(waypoint)
            rospy.loginfo("WaypointUpdater.listSomeWaypoints-"+str(w)+": "+str(waypointDictionary))
            w+=1

    def waypoints_cb(self, lane):
        if (UNITTESTINGWAYPOINTUPDATER):
            self.testIsPoseInFront()
            self.testFindFirstWaypointInFront()
        # TODO: Implement
        headerDictionary = message_converter.convert_ros_message_to_dictionary(lane.header)
        rospy.loginfo("WaypointUpdater.waypoints_cb-type(lane):"+str(type(lane))+", len(lane.waypoints):"+str(len(lane.waypoints)))
        rospy.loginfo("WaypointUpdater.waypoints_cb-lane.header:"+str(headerDictionary))
        self.lane=lane
        if UNITTESTINGWAYPOINTUPDATER:
            self.listSomeWaypoints(lane.waypoints[:5])
            self.listSomeDistancesBetweenWaypoints(self.lane.waypoints, [0,5])
  
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distanceToWaypoint(self, vehiclePosition, vehicleYaw, waypointPosition):
        #rospy.loginfo("WaypointUpdater.distanceToWaypoint-tye(vehiclePosition):"+str(type(vehiclePosition))+
        #        ", type(waypointPosition):"+str(type(waypointPosition)))
        #rospy.loginfo("WaypointUpdater.distanceToWaypoint-vehiclePosition:"+str(vehiclePosition)+
        #        ", waypointPosition:"+str(waypointPosition))
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        signOfDistance=1. if self.isWaypointInFront(vehiclePosition, vehicleYaw, waypointPosition) else -1.
        return signOfDistance*dl(vehiclePosition, waypointPosition)        

if __name__ == '__main__':
    try:
        if (UNITTESTINGWAYPOINTUPDATER):
            testMetersToMiles()
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
