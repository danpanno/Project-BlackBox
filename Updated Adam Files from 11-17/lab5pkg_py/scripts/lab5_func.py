#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab5_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	

	C = 0.001 #M and  orignally done in mm, used C = 0.001 to convert all values to m.

	M = np.array([[0,-1,0,390*C], [0, 0, -1, 401*C], [1, 0,0, 215.5*C], [0, 0, 0, 1]])

	w1 = np.array([[0,0,1]])
	w2 = np.array([[0,1,0]])
	w3 = np.array([[0,1,0]])
	w4 = np.array([[0,1,0]])
	w5 = np.array([[1,0,0]])
	w6 = np.array([[0,1,0]])

	q1 = np.array([[-150,150,0]])*C #converts from mm to m.
	q2 = np.array([[-150,0,162]])*C
	q3 = np.array([[94,0,162]])*C
	q4 = np.array([[307,0,162]])*C
	q5 = np.array([[0,260,162]])*C
	q6 = np.array([[390,0,162]])*C
	
	v1 = np.cross(-w1,q1)
	v2 = np.cross(-w2,q2)
	v3 = np.cross(-w3,q3)
	v4 = np.cross(-w4,q4)
	v5 = np.cross(-w5,q5)
	v6 = np.cross(-w6,q6)

	S1 = np.hstack((w1, v1))
	S1 = S1.T 
	S2 = np.hstack((w2, v2))
	S2 = S2.T
	S3 = np.hstack((w3, v3))
	S3 = S3.T
	S4 = np.hstack((w4, v4))
	S4 = S4.T
	S5 = np.hstack((w5, v5))
	S5 = S5.T
	S6 = np.hstack((w6, v6))
	S6 = S6.T

	S = np.hstack((S1, S2, S3, S4, S5, S6))



	
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	#print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	#T = np.eye(4)

	M, S = Get_MS()

	bS1 = np.array([[0,-S[2][0],S[1][0], S[3][0]], [S[2][0], 0, -S[0][0], S[4][0]], [-S[1][0], S[0][0], 0, S[5][0]], [0, 0, 0, 0] ])
	bS2 = np.array([[0,-S[2][1],S[1][1], S[3][1]], [S[2][1], 0, -S[0][1], S[4][1]], [-S[1][1], S[0][1], 0, S[5][1]], [0, 0, 0, 0] ])
	bS3 = np.array([[0,-S[2][2],S[1][2], S[3][2]], [S[2][2], 0, -S[0][2], S[4][2]], [-S[1][2], S[0][2], 0, S[5][2]], [0, 0, 0, 0] ])
	bS4 = np.array([[0,-S[2][3],S[1][3], S[3][3]], [S[2][3], 0, -S[0][3], S[4][3]], [-S[1][3], S[0][3], 0, S[5][3]], [0, 0, 0, 0] ])
	bS5 = np.array([[0,-S[2][4],S[1][4], S[3][4]], [S[2][4], 0, -S[0][4], S[4][4]], [-S[1][4], S[0][4], 0, S[5][4]], [0, 0, 0, 0] ])
	bS6 = np.array([[0,-S[2][5],S[1][5], S[3][5]], [S[2][5], 0, -S[0][5], S[4][5]], [-S[1][5], S[0][5], 0, S[5][5]], [0, 0, 0, 0] ])

	E1 = expm(bS1*theta1)
	E2 = expm(bS2*theta2)
	E3 = expm(bS3*theta3)
	E4 = expm(bS4*theta4)
	E5 = expm(bS5*theta5)
	E6 = expm(bS6*theta6)

	T = np.matmul(E1,E2)
	T = np.matmul(T,E3)
	T = np.matmul(T,E4)
	T = np.matmul(T,E5)
	T = np.matmul(T,E6)

	T = np.matmul(T,M)

	#print T




	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	#assign lengths for arm segments
	L1 = 0.152
	L2 = 0.120
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059

	#calculate the transformation matrix from world frame to base frame
	#-150 mm in X
	#150 mm in Y
	#10 mm in Z
	Twb = np.array([[1, 0, 0, -0.150], [0, 1, 0, 0.150], [0, 0, 1, 0.010], [0, 0, 0, 1]])

	#calculate the transformation matrix from base frame to world frame
	Tbw = np.array([[1, 0, 0, 0.150], [0, 1, 0, -0.150], [0, 0, 1, -0.010], [0, 0, 0, 1]])
	#can also just do inverse of matrix Twb

	#Step 1: convert the user given coordinates from world frame to base frame
	wcoords = np.array([[xWgrip, yWgrip, zWgrip, 1]]).T #append 1 for multiplication and define as a column vector
	wcoords = wcoords.astype(float) #converts the matrix elements as 'objects' to float
	bcoords = np.matmul(Tbw,wcoords)
	xgrip = bcoords[0][0]
	ygrip = bcoords[1][0]
	zgrip = bcoords[2][0]
	
	#convert yaw from degrees to radians
	yaw = np.radians(yaw_WgripDegree)

	#Step 2: determine the wrist's center point <xcen, ycen, zcen> in terms of <xgrip, ygrip, zgrip> and yaw
	xcen = xgrip - L9*np.cos(yaw)
	ycen = ygrip - L9*np.sin(yaw)
	zcen = zgrip

	#Step 3: determine theta1 with <xcen, ycen, zcen>. ensure atan2() is used. account for the 27mm offset to the 3end point
	alpha = np.arctan2(ycen,xcen) #atan2(ycen/xcen)
	beta = np.arcsin((L6 + 0.027)/(np.sqrt(xcen**2 + ycen**2))) #** is the exponent/power operator
	theta1 = alpha - beta

	#Step 4: determine theta6, given yaw and theta1
	theta6 = theta1 + (np.pi)/2 - yaw

	#Step 5: determine the projected end point <x3end, y3end, z3end>
	#calculate Tbc from the base frame to center point frame
	Tbc = np.array([[np.cos(theta1), -np.sin(theta1), 0, xcen], [np.sin(theta1), np.cos(theta1), 0, ycen], [0, 0, 1, zcen], [0, 0, 0, 1]])
	#determine the translation vector from center point frame origin to 3end point
	pc3end = np.array([[-L7, -(L6 + 0.027), L8 + L10, 1]]).T #append 1 for multiplication and define as a column vector
	#determine the coordinates for the projected end point
	pb3end = np.matmul(Tbc,pc3end)
	x3end = pb3end[0][0]
	y3end = pb3end[1][0]
	z3end = pb3end[2][0]

	#Step 6: determine theta2, theta3, and theta4 in terms of the projected end point.
	#use the rule of cosines and intermediate variables to determine these angles.
	
	#determine theta2
	#intermediate lengths
	z2 = z3end - L1
	#calculate the length of the vector in the x-y plane from the base 
	#to the projected end point
	L_end = np.sqrt(x3end**2 + y3end**2)
	h = np.sqrt(L_end**2 + z2**2)
	#intermediate angles
	phi1 = np.arccos(L_end/h)
	phi2 = np.arccos((L3**2 + h**2 - L5**2)/(2*L3*h))
	#calculate theta2
	theta2 = -(phi1 + phi2)

	#determine theta4
	#intermediate angle
	phi4 = np.arccos((L5**2 + h**2 - L3**2)/(2*L5*h))
	#calculate theta4
	theta4 = -(phi4 - phi1)

	#calculate theta3
	theta3 = -theta4 - theta2

	#define theta5
	#for our use case, theta5 will always be -90 degrees
	theta5 = -np.pi/2

	#call lab_fk using the calculated angles
	radangles = lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
	
	#return the offset adjusted lab_fk angles
	return radangles 




	# ==============================================================#
	pass
   


