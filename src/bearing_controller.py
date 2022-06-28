#!/usr/bin/python

import rospy
from math import sin,cos,sqrt,atan2,acos,pi
import numpy as np
import gurobipy as gp
from scipy.optimize import minimize
from geometry_msgs.msg import Twist
from px4_mavros import Px4Controller
from std_msgs.msg import Float64MultiArray
from gurobipy import GRB

P1,P2,P3,Pc,Pr,Pb,A,b,Pcen = None,None,None,None,None,None,None,None,None
bearing_cmd_vel = Twist()
sigma_u,sigma_v,sigma_ranging,sigma_bearing,sigma_alpha = 0.007,0.007,0.01,0.01,0.01
'''
height_l = 0.2 #exp
height_u = 0.5 # 
d_safe_car = 0.7 # 
d_measuring = 1.5 # 
'''
height_l = 0.5 #sim
height_u = 2 #
d_safe_car = 2 #
d_measuring = 7 #
d_safe_uav = 1
d_communication = 20
m,x = None,None
gamma,gain = 1,1

def odom(msg):
	global P1,P2,P3,Pc,Pr,Pb,A,b,Pcen
	
	Pc = np.array(msg.data[18:21])
	Pr = np.array(msg.data[21:24])
	Pb = np.array(msg.data[24:27])
	P1 = np.array(msg.data[0:3])
	P2 = np.array(msg.data[6:9])
	P3 = np.array(msg.data[12:15])
	Pcen = (P1+P2+P3)/3

	A = np.array([ \
				  (-2*(Pb-P1)[:2]).tolist()+[0], \
				  (-2*(Pb-P2)[:2]).tolist()+[0], \
				  (-2*(Pb-P3)[:2]).tolist()+[0], \
				  (2*(Pb-P1)[:2]).tolist()+[0], \
				  (2*(Pb-P2)[:2]).tolist()+[0], \
				  (2*(Pb-P3)[:2]).tolist()+[0], \
				  (-2*(Pb-Pc)[:2]).tolist()+[0], \
				  (-2*(Pb-Pr)[:2]).tolist()+[0], \
				  (2*(Pb-Pc)[:2]).tolist()+[0], \
				  (2*(Pb-Pr)[:2]).tolist()+[0], \
				  [0]*2+[-1], \
				  [0]*2+[1], \
				  ])

	b = np.array([ \
				  np.linalg.norm((Pb-P1)[:2])**2 - d_safe_car**2, \
				  np.linalg.norm((Pb-P2)[:2])**2 - d_safe_car**2, \
				  np.linalg.norm((Pb-P3)[:2])**2 - d_safe_car**2, \
				  d_measuring**2 - np.linalg.norm((Pb-P1)[:2])**2, \
				  d_measuring**2 - np.linalg.norm((Pb-P2)[:2])**2, \
				  d_measuring**2 - np.linalg.norm((Pb-P3)[:2])**2, \
				  np.linalg.norm((Pb-Pc)[:2])**2 - d_safe_uav**2, \
				  np.linalg.norm((Pb-Pr)[:2])**2 - d_safe_uav**2, \
				  d_communication**2 - np.linalg.norm((Pb-Pc)[:2])**2, \
				  d_communication**2 - np.linalg.norm((Pb-Pr)[:2])**2, \
				  gain*(Pb[2] - height_l), \
				  gain*(height_u - Pb[2]) \
				  ])*gamma

def qp_ini():
	global m,x
	
	m = gp.Model("qp")
	m.setParam("NonConvex", 2.0)
	m.setParam("LogToConsole",0)
	x = m.addVars(3,ub=0.3, lb=-0.3, name="x")

def addCons(i):
	global m

	m.addConstr(A[i,0]*x[0] + A[i,1]*x[1] + A[i,2]*x[2] <= b[i], "c"+str(i))
	
def	qpsolver():
	global bearing_cmd_vel,x
	
	#obj = -(x[0] - (P1 - Pb)[0])**2 - (x[1] - (P1 - Pb)[1])**2 - (x[2] - (P1 - Pb)[2])**2 - (x[0] - (P2 - Pb)[0])**2 - (x[1] - (P2 - Pb)[1])**2 - (x[2] - (P2 - Pb)[2])**2 - (x[0] - (P3 - Pb)[0])**2 - (x[1] - (P3 - Pb)[1])**2 - (x[2] - (P3 - Pb)[2])**2 # worst
	obj = (x[0] - (P1 - Pb)[0])**2 + (x[1] - (P1 - Pb)[1])**2 + (x[2] - (P1 - Pb)[2])**2 + (x[0] - (P2 - Pb)[0])**2 + (x[1] - (P2 - Pb)[1])**2 + (x[2] - (P2 - Pb)[2])**2 + (x[0] - (P3 - Pb)[0])**2 + (x[1] - (P3 - Pb)[1])**2 + (x[2] - (P3 - Pb)[2])**2 # optimal
	m.setObjective(obj)

	m.remove(m.getConstrs())
	
	for i in range (b.size):
		addCons(i)

	m.optimize()
	optimal = m.getVars()
	#print(A.dot(np.array([optimal[0].X,optimal[1].X,optimal[2].X])) - b)
	
	bearing_cmd_vel.linear.x = optimal[0].X
	bearing_cmd_vel.linear.y = optimal[1].X
	bearing_cmd_vel.linear.z = optimal[2].X

	px4_bearing.vel_control(bearing_cmd_vel)

if __name__ == '__main__':
	try:
		rospy.init_node('bearing_controller')
		px4_bearing = Px4Controller("iris_bearing")
		rospy.Subscriber('/state', Float64MultiArray, odom, queue_size=10)
		rate = rospy.Rate(100)
		while b is None:
			rate.sleep()

		qp_ini()
		while not rospy.is_shutdown():
			qpsolver()
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
