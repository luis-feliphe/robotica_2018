# -*- coding: utf-8 -*-
#follower
#cmd_vel,odom 
#Folow robot 1 if it exists
#turlebot
#./know/follower.py
#name
#controlo 
#active


######################################
# This file simulate a robot on ROS. #
# To use, you need to pass like      #
# argument the numnber of robot,     #
# like "./movingRobot 1"             #
######################################


from Controlo import Controlo
#ROS  imports
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import String
import os
import random
import sys
import time
#extra
from kobuki_msgs.msg import Sound
from kobuki_msgs.msg import Led
####
from tf.transformations import euler_from_quaternion
import tf
import struct
from datetime import datetime
from sensor_msgs.msg import LaserScan
getTime = lambda: int(round(time.time() * 1000))

import math
RATE = 6 

TEMPO_SEGUINDO = 40
global posicao
posicao = None
global leaderpos
leaderpos = None
global contPontos
contPontos = 0

global leaderInPos
leaderInPos = False

#if leader in position or in the way
def leaderInPosition():
	global leaderInPos
	global leaderpos
	x, y, z = getxy(leaderpos)
	u = 1.5
	if leaderInPos: 
		return True
	else:
		if (x >= -u/2.0 and x <= u/2.0):
			if (y<= -1.5 * u and y>= -2.5*u):
				leaderInPos = True
				return True

	return False
def leaderInFinal():
	global leaderpos
	x, y, z = getxy(leaderpos)
	u = 1.5
	if (x >= -u/2.0 and x <= u/2.0):
		if (y>= 1.5 * u and y<= 2.5*u):
			return True
	return False

def hasLeaderPos():
	global leaderpos
	return leaderpos != None

def degrees(value):
	return (value*180)/math.pi#math.degrees(value)#((value* 180.0)/math.pi)
def getpos(odom):
	global posicao
	posicao= odom

def getleaderpos(odom):
	global leaderpos
	leaderpos= odom
def hasDataToWalk():
	global posicao
	return posicao != None

def getDataFromRos():
	global posicao
	global leaderpos
	global myId
	global contPontos
	x, y, z = getxy(leaderpos)
	mx, my, mz = getxy (posicao)
	x = x + 0.75
	y = y - 0.75
	return x, y, z, mx, my, mz

def my_sector (odom):
        u = 1.5
        div = 2.0
        global resultado
        x,y = round (odom.pose.pose.position.x, 2), round ( odom.pose.pose.position.y,2)
        if (x >= u/div and x <= 1.5*u):
                if (y >= u/div and y<= 1.5*u):
                        resultado = "G"
                elif (y>= -u/div and y < u/div):
                        resultado = "H"
                elif (y>= -1.5 * u and y < -u/div):
                        resultado = "I"
                else:
                        resultado = "Muito abaixo ou acima"
        elif (x> -u/div and x < u/div):
                if (y >= u/div and y<= 1.5*u):
                        resultado = "F"
                elif (y>= -2.5 * u and y < -1.5* u):
                        resultado = "J"
                else:
                        resultado = "Muito abaixo ou acima"
        elif (x> -1.5*u and x < -u/div):
                if (y >= u/div and y<= 1.5*u):
                        resultado = "E"
                elif (y>= -u/div and y < u/div):
                        resultado = "D"
                else:
                        resultado = "Muito abaixo ou acima"
        else:
                resultado = "nao encontrado"
	return resultado



def getDegreesFromOdom(w):
	#TODO: HOW CONVERT DATA TO ANGLES
	q = [w.pose.pose.orientation.x,	w.pose.pose.orientation.y, w.pose.pose.orientation.z, w.pose.pose.orientation.w]       
        euler_angles = euler_from_quaternion(q, axes='sxyz')
	current_angle = euler_angles[2]
	if current_angle < 0:
		current_angle = 2 * math.pi + current_angle
	return degrees(current_angle)
		

def getxy (odom):
	return round (odom.pose.pose.position.x, 2), round ( odom.pose.pose.position.y,2), round (getDegreesFromOdom (odom),2)#degrees(yall)

#############
# ROS SETUP #
#############
#Became a node, using the arg to decide what the number of robot
global myId
myId = sys.argv[1].replace("robot_", "")
	
robot = sys.argv[1]#,sys.argv[2],sys.argv[3], sys.argv[4]
rospy.init_node("robot_"+str(robot)+"_folower")
robot = None
finish = None
p = None

if False: #simulated robots
	rospy.Subscriber(robot+"/odom", Odometry, getpos)
	rospy.Subscriber("/robot_0/odom", Odometry, getleaderpos)
	finish = rospy.Publisher(robot+"/working", Bool)
	p = rospy.Publisher(robot+"/cmd_vel", Twist)
else: # real robots
	rospy.Subscriber(robot+"/odom", Odometry, getpos)
	rospy.Subscriber("/robot_0/odom", Odometry, getleaderpos)
	finish = rospy.Publisher(robot+"/working", Bool)
	p = rospy.Publisher(robot+"/cmd_vel_mux/input/teleop", Twist)

r = rospy.Rate(RATE) # 5hz

cont = 0
posInicialx=0
posInicialy=0

print "Iniciado o follower"
#### Iniciando o loop principal ######
tempoInicial = getTime()
cont = 1
try:
	algoritmo = Controlo()
	while not rospy.is_shutdown():
		if hasDataToWalk() and hasLeaderPos():
			global contPontos
			x, y, z, mx, my, mz = getDataFromRos()
			t= Twist()
#			print "Indo para " + str (x) + ":" + str (y) + ":" + str (z) +  " Estando em " + str (my)+ ":" +  str (my) + ":" +  str (my)
			lin,ang  = algoritmo.start(x,y, z, mx, my, mz)
			t.angular.z = ang
			t.linear.x = lin
			if (lin == 0 and ang == 0):
				contPontos = contPontos + 1
			p.publish(t)

		if hasLeaderPos() and leaderInFinal() and ang == 0  and lin == 0:
			print "Chegou a posicao final"
			#If leader in position and we do not need to move, we are in final position
			finish.publish(False)
			sys.exit()


		r.sleep()

except Exception :
	raise	
	print ("Exception!\n")

