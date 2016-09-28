#!/usr/bin/env python2

import rospy
import numpy as np
import unittest
import time as t
from collections import defaultdict
from scipy.linalg import block_diag
#Message types
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from cylinder.msg import cylDataArray
from cylinder.msg import cylMsg
#Functions
from Relative2AbsolutePose import Relative2AbsolutePose
from Relative2AbsoluteXY import Relative2AbsoluteXY
from Absolute2RelativeXY import Absolute2RelativeXY
from pi2pi import pi2pi
from mapping import mapping

#landmarks' most recent absolute coordinate
landmark_abs_ = defaultdict(list)
seenLandmarks_ =[]
#State Transition Model
F_ = []
#Control-Input Model
W_ = []
# dimension of robot pose
dimR_ = 3

class Robot ():

    def __init__ (self, pose, pos_Cov, sense_Type):

        self.x = pose[0][0]
        self.y = pose[1][0]
        self.theta = pose[2][0]
        self.poseCovariance = pos_Cov
        self.senseType = sense_Type

    def setPose (self,new_pose):

        self.x = new_pose[0][0]
        self.y = new_pose[1][0]
        self.theta = new_pose[2][0]

    def getPose(self):

        return [[self.x], [self.y], [self.theta]]


    def setCovariance (self, new_Cov):

        self.posCov = new_Cov

    def getCovariance (self):

        return self.poseCovariance

    def move (self, robotCurrentAbs, u):

        [nextRobotAbs, H1, H2] = Relative2AbsolutePose(robotCurrentAbs,u)

        self.x = nextRobotAbs[0][0]
        self.y = nextRobotAbs[1][0]
        self.theta = nextRobotAbs[2][0]
        return nextRobotAbs, H1, H2

   def sense (self, robotCurrentAbs, landmarkAbs):

        if self.senseType == 'Vision':
            [measurement, H1, H2] = Absolute2RelativeXY(robotCurrentAbs, landmarkAbs)

        else:
            raise ValueError ('Unknown Measurement Type')

        return measurement, H1, H2

    def inverseSense (self, robotCurrentAbs, measurement):

        if self.senseType == 'Vision':
            [landmarkAbs, H1, H2] = Relative2AbsoluteXY(robotCurrentAbs, measurement)

        else:
            raise ValueError ('Unknown Measurement Type')

        return landmarkAbs, H1, H2

class LandmarkMeasurement ():

    def __init__ (self, meas_Cov):

        self.measurementCovariance = meas_Cov

    def setCovariance (self,new_Cov):

        self.measurementCovariance = new_Cov

    def getCovariance (self):

        return self.measurementCovariance

class Motion ():

    def __init__ (self, motion_command, motion_Cov):

        self.u = motion_command
        self.motionCovariance = motion_Cov

    def setCovariance (self, new_Cov):

        self.motionCovariance = new_Cov

    def getCovariance (self):

        return self.motionCovariance

    def setMotionCommand (self, motionCommand):

        self.u = motionCommand

    def getMotionCommand (self):

        return self.u


class KalmanFilter(Robot):

    def __init__ (self, mean, covariance, robot):

        self.stateMean = mean
        self.stateCovariance = covariance
        self.robot = robot

    def setStateMean (self, mean):

        self.stateMean = mean

    def getStateMean (self):

        return self.stateMean

    def setStateCovariance (self, covariance):

        self.stateCovariance = covariance

    def getStateCovariance (self):

        return self.stateCovariance

    def predict (self,motion, motionCovariance):
        # TODO get robot current pose
        x,y,theta=robot.getPose()
        # TODO move robot given current pose and u

        # TODO predict state mean
        # TODO predict state covariance
        # TODO set robot new pose
        # TODO set robot new covariance
        # TODO set KF priorStateMean
        # TODO set KF priorStateCovariance

        return priorStateMean, priorStateCovariance

    def update(self,measurement, measurementCovariance, new):
        global seenLandmarks_
        global dimR_
        # TODO get robot current pose
        # get landmark absolute position estimate given current pose and measurement (robot.sense)
        [landmarkAbs, G1, G2] = self.robot.inverseSense(robotCurrentAbs, measurement)
        # TODO get KF state mean and covariance
        # if new landmark augment stateMean and stateCovariance
        if new:
            stateMean = np.concatenate((stateMean,[[landmarkAbs[0]], [landmarkAbs[1]]]),axis = 0)
            Prr = self.robot.getCovariance()
            if len(seenLandmarks_) == 1:
                Plx = np.dot(G1,Prr)
            else:
                lastStateCovariance    = KalmanFilter.getStateCovariance(self)
                Prm    = lastStateCovariance[0:3,3:]
                Plx    = np.dot(G1, np.bmat([[Prr, Prm]]))
            Pll = np.array(np.dot(np.dot(G1, Prr),np.transpose(G1))) + np.array(np.dot(np.dot(G2, measurementCovariance),np.transpose(G2)))
            P = np.bmat([[stateCovariance, np.transpose(Plx)],[Plx,Pll]])
            stateCovariance = P
        # else:
        # if old landmark stateMean & stateCovariance remain the same (will be changed in the update phase by the kalman gain)
        # TODO calculate expected measurement
        # get measurement
        Z = ([ [measurement[0]],[measurement[1]] ])
        #Update
        x = stateMean
        # TODO y = Z - expectedMeasurement
        # build H
        # H = [Hr, 0, ..., 0, Hl] position of Hl depends on when was the landmark seen?
        H = np.reshape(Hr, (2, 3))
        for i in range(0, len(seenLandmarks_)-1):
            H = np.bmat([H, np.zeros([2,2]) ])
        H = np.bmat([H, np.reshape(Hl, (2, 2))])
        # TODO compute S
        if (S < 0.000001).all():
            print('Non-invertible S Matrix')
            raise ValueError
            return
        else:
        # TODO calculate Kalman gain
        # TODO compute posterior mean
        # TODO compute posterior covariance
        # check theta robot is a valid theta in the range [-pi, pi]
        posteriorStateMean[2][0] = pi2pi(posteriorStateMean[2][0])
        # TODO update robot pose
        # set robot pose
        self.robot.setPose(robotPose)
        # TODO updated robot covariance
        # set robot covariance
        self.robot.setCovariance(robotCovariance)
        # set posterior state mean
        KalmanFilter.setStateMean(self,posteriorStateMean)
        # set posterior state covariance
        KalmanFilter.setStateCovariance(self,posteriorStateCovariance)
        print 'robot absolute pose : ',robotAbs
        label = measurement[2]
        vec = mapping(seenLandmarks_.index(label)+1)
        landmark_abs_[int(label)-1].append([[stateMean[dimR_ + vec[0]-1][0]],[stateMean[dimR_ + vec[1]-1][0]]])
        for i in range(0,len(landmark_abs_)):
            print 'landmark absolute position : ',i+1,',',np.median(landmark_abs_[i],0)
        return posteriorStateMean, posteriorStateCovariance

class SLAM(LandmarkMeasurement, Motion, KalmanFilter):

    def callbackOdometryMotion(self, msg):
        # TODO read msg received
        # TODO You can choose to only rely on odometry or read a second sensor measurement
        # TODO compute dt = duration of the sensor measurement
        # TODO compute command
        # set motion command
        self.motion.setMotionCommand(u)
        # get covariance from msg received
        covariance = msg.twist.covariance
        self.motion.setCovariance([[covariance[0],0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,covariance[35]]])
        poseCovariance = self.robot.getCovariance()
        # call KF to execute a prediction
        self.KF.predict(self.motion.getMotionCommand(), self.motion.getCovariance())

    def callbackLandmarkMeasurement(self, data):
        global seenLandmarks_
        for i in range(0,len(data.cylinders)):
            # read data received
            # aligning landmark measurement frame with robot frame
            dx = data.cylinders[i].Zrobot
            dy = -data.cylinders[i].Xrobot
            label = data.cylinders[i].label
            # determine if landmark is seen for first time
            # or it's a measurement of a previously seen landamrk
            new = 0
            # if seenLandmarks_ is empty
            if not seenLandmarks_:
                new = 1
                seenLandmarks_.append(label)
            # if landmark was seen previously
            elif label not in seenLandmarks_:
                new = 1
                seenLandmarks_.append(label)
            measurement = []
            measurement.append(dx)
            measurement.append(dy)
            measurement.append(label)
            # get covariance from data received
            covariance = data.cylinders[i].covariance
            self.landmarkMeasurement.setCovariance([[covariance[0],0.0],[0.0, covariance[3]]])
            measurementLandmarkCovariance = self.landmarkMeasurement.getCovariance()
            # call KF to execute an update
            try:
                self.KF.update(measurement, measurementLandmarkCovariance, new)
            except ValueError:
                return

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Initialise robot
        # TODO initialise a robot pose and covariance
        sense_Type = 'Vision'
        self.robot = Robot(robot_pose, robot_covariance, sense_Type)
        # Initialise motion
        # TODO initialise a motion command and covariance
        self.motion = Motion(motion_command, motion_covariance)
        # Initialise landmark measurement
        # TODO initialise a measurement covariance
        self.landmarkMeasurement = LandmarkMeasurement(measurement_covariance)
        # Initialise kalman filter
        # TODO initialise a state mean and covariance
        # initial state contains initial robot pose and covariance
        self.KF = KalmanFilter(state_mean, state_mean, self.robot)
        # TODO Subscribe to different topics and assign their respective callback functions
        rospy.spin()

if __name__ == '__main__':
    print('Landmark SLAM Started...')
    # Initialize the node and name it.
    rospy.init_node('listener')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        slam = SLAM()
    except rospy.ROSInterruptException: pass
