#!/usr/bin/env python

import rospy, sys, select, termios, tty,os
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from Relative2AbsolutePose import Relative2AbsolutePose as r2a
from Relative2AbsoluteXY import Relative2AbsoluteXY as r2xy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from std_msgs.msg import String
from cylinder.msg import cylDataArray
from cylinder.msg import  cylMsg

# reads the key that has been pressed
def getKey():
    '''returns the key that has been pressed on the keyboard'''
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    #return terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#class for determining robot and landmark positions
class RobotPositionCalculator:
    def __init__(self):
        #Robot Odometry Subscriber
        self.odom_sub = rospy.Subscriber('odom',Odometry,self.robotCallback)
        self.cylinder_sub =rospy.Subscriber("/cylinderTopic",cylDataArray,self.landmarkCallback)
        #initialise robot position variables
        self.robot_pos = [[0.],[0.],[0.]]
        self.robot_hist = self.robot_pos
        self.velocity = [[0.],[0.],[0.]]

        #intiialise measurement file
        self.MeasfileName = 'MesurementFile.txt'
        self.MeasfilePath=os.path.join(os.path.expanduser('~'),'Documents',self.MeasfileName)
        self.MeasrecordFile = open(self.MeasfilePath,'w+')
        self.MeasrecordFile.close()
        #initialise points files
        self.PointsfileName='PointsFile.txt'
        self.PointsfilePath=os.path.join(os.path.expanduser('~'),'Documents',self.PointsfileName)
        self.PointsrecordFile = open(self.PointsfilePath,'w+')
        self.PointsrecordFile.close()
        #paramter for tracking first data
        self.firstTwist=True
        #cylinder data listing
        self.cyls=cylinderData()
        #constant parameters
        self.ROBOT=1
        self.LANDMARK=0

    def robotCallback(self,msg):
        '''Robot Odometry subscriber and posiiton calculator'''
        #time interval
        currentTime=(msg.header.stamp.secs+msg.header.stamp.nsecs*10e-9)
        #setup first time
        if self.firstTwist==True:
            self.previousTime=currentTime
            self.velocity[0][0]=msg.twist.twist.linear.x
            self.velocity[1][0]=msg.twist.twist.linear.y
            self.velocity[2][0]=msg.twist.twist.angular.z
            self.firstTwist=False

        #make sure the data is current
        if currentTime>self.previousTime:
            #calculate the ime between received data
            deltaTime=currentTime-self.previousTime

            #store time
            self.previousTime=currentTime

            #store the measurement data
            self.measurementStore(self.ROBOT,msg)

            #calculate local-frame movement from last
            u=[[0.],[0.],[0.]]

            #calculate local-frame movement from last
            u[0][0]=(self.velocity[0][0]+msg.twist.twist.linear.x)/2.0*deltaTime
            u[1][0]=(self.velocity[1][0]+msg.twist.twist.linear.y)/2.0*deltaTime
            u[2][0]=(self.velocity[2][0]+msg.twist.twist.angular.z)/2.0*deltaTime

            #update velocity
            self.velocity[0][0]=msg.twist.twist.linear.x
            self.velocity[1][0]=msg.twist.twist.linear.y
            self.velocity[2][0]=msg.twist.twist.angular.z

            #calculate the robot position
            self.robot_pos=r2a(self.robot_pos,u)
            self.robot_hist[0].append(self.robot_pos[0][0])
            self.robot_hist[1].append(self.robot_pos[1][0])
            self.robot_hist[2].append(self.robot_pos[2][0])

            #store the robot position
            self.positionStore(self.ROBOT,msg,str(currentTime))

    def landmarkCallback(self,msg):
        '''process data for each of the cylinders and determine the position'''
        for cylinder in msg.cylinders:
            #store the measurement data
            self.measurementStore(self.LANDMARK,cylinder)
            #calculate distance
            landmark_pos=r2xy(self.robot_pos,[cylinder.Zrobot,cylinder.Xrobot])
            #store position data
            self.positionStore(self.LANDMARK,landmark_pos,cylinder.label)
            #add data to cylinder array
            self.cyls.updateCylinder(cylinder.label,landmark_pos[0],landmark_pos[1])

    def positionStore(self,item,msg,label):
        '''Write the position of robot or landmark to file'''
        #store the robot position
        self.PointsrecordFile=open(self.PointsfilePath,'a')
        if item==self.ROBOT:
            self.PointsrecordFile.write('POSE2D' + ',' + str(self.robot_pos[0][0]) + ',' + str(self.robot_pos[1][0]) + ',' + str(self.robot_pos[2][0]) + ',' + str(msg.pose.pose.position.x) + ',' + str(msg.pose.pose.position.y) + '\n')
        elif item==self.LANDMARK:
            self.PointsrecordFile.write('POINT2D' + ',' + str(label) + ',' + str(msg[0]) + ',' + str(msg[1]) + '\n')
        self.PointsrecordFile.close()

    def measurementStore(self,item,msg):
        '''Write the measurement data of robot or landmark to file'''
        #store the measurement data
        self.MeasrecordFile=open(self.MeasfilePath,'a')
        if item==self.ROBOT:
            self.MeasrecordFile.write('ODOMETRY_MEAS2D' + ',' + str(msg.twist.twist.linear.x) + ',' + str(msg.twist.twist.linear.y) + ',' + str(msg.twist.twist.angular.z) + ',' + str(msg.twist.covariance[0]) + ',' + str(msg.twist.covariance[1]) + ',' + str(msg.twist.covariance[5]) + ',' + str(msg.twist.covariance[7]) + ',' + str(msg.twist.covariance[15]) + ',' + str(msg.twist.covariance[35]) + '\n')
        elif item==self.LANDMARK:
            self.MeasrecordFile.write('LANDMARK_MEAS2D' + ',' + str(msg.Zrobot) + ',' + str(msg.Xrobot) + ',' + str(msg.covariance[0]) + ',' + str(msg.covariance[1]) + ',' + str(msg.covariance[3]) + '\n')
        self.MeasrecordFile.close()

class cylinderData:
    def __init__(self):
        self.cylinderArray=[]
        self.cylinderCount=0

    def inSet(self,cylinderNum):
        '''determine if cylinder is in data set'''
        count=0
        for cylinder in self.cylinderArray:
            if cylinder.label==cylinderNum:
                return True,count
            else:
                count=count+1
        return False,0

    def getCylinderInfo(self,cylinderNum):
        '''retrieve cylinder information'''
        Valid,ref=self.inSet(cylinderNum)
        if Valid==True:
            return self.cylinderArray[ref].label,self.cylinderArray[ref].Xposition,self.cylinderArray[ref].yposition, self.cylinderArray[ref].count
        else:
            return 0,0,0,0

    def updateCylinder(self,cylinderRef,x,y):
        '''update cylinder information with observation data'''
        Valid,ref=self.inSet(cylinderRef)
        if Valid==True:
            self.cylinderArray[ref].updateCylinder(x,y)
        else:
            self.addCylinder(cylinderRef,x,y)

    def addCylinder(self,label,x,y):
        '''add a new cylinder to the data set'''
        #check that cylidner is in data set
        inSet,ref=self.inSet(label)
        if inSet==True:
            print 'already in set'
        else:
            newCylinder=Cylinder(label,x,y)
            self.cylinderArray.append(newCylinder)
            self.cylinderCount=self.cylinderCount+1

class Cylinder:
    def __init__(self,cylLabel,xPos,yPos):
        self.label=cylLabel
        self.Xposition=xPos
        self.Yposition=yPos
        self.count=1

    def updateCylinder(self,x,y):
        '''update cylinder information with observation data'''
        self.Xposition=(self.Xposition*self.count+x)/(self.count+1)
        self.Yposition=(self.Yposition*self.count+y)/(self.count+1)
        self.count=self.count+1


if __name__=="__main__":
    '''reads the odometry information and determines the robots absolute position in the global frame'''

    #get the terminal settings (for post key-capture)
    settings = termios.tcgetattr(sys.stdin)

    #initiliase this node
    rospy.init_node('turtlebotposition')

    try:
        fileName='MeasuremntFile.txt'

        robotPosition=RobotPositionCalculator()

        #enter an infinite loop
        while not rospy.is_shutdown():

            #get the key value
            key = getKey()

            #if  stop pressed
            if key == '\x03':
                break
            #plot the current map
            elif key == 'p':
                #setup plot
                CylindersX=[]
                CylindersY=[]

                for cylinder in robotPosition.cyls.cylinderArray:
                    CylindersX.append(cylinder.Xposition)
                    CylindersY.append(cylinder.Yposition)
                plt.plot(robotPosition.robot_hist[0],robotPosition.robot_hist[1],'g-',CylindersX,CylindersY,'ro')
                plt.ylabel('Y (m)')
                plt.xlabel('X (m)')
                plt.title('Robot and Landmark Position')
                for cylinder in robotPosition.cyls.cylinderArray:
                #plot cylinder labels
                    plt.text(cylinder.Xposition,cylinder.Yposition,'Cylinder ' + str(cylinder.label))
                plt.show()

    except Exception as e:
        print e

    #if something doesn't work go to zero
    finally:
        print 'exiting now'
    #return the terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
