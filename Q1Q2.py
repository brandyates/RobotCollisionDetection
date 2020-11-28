#Created for CAP 4662. Most code by Tianze Chen, Number 1 and2 by Brandon Yates
import sys
import numpy as np
sys.path.append('PythonAPI')
import math
import time
try:    
    import sim
except:    
    print ('--------------------------------------------------------------')    
    print ('"sim.py" could not be imported. This means very probably that')   
    print ('either "sim.py" or the remoteApi library could not be found.')   
    print ('Make sure both are in the same folder as this file,')    
    print ('or appropriately adjust the file "sim.py"')    
    print ('--------------------------------------------------------------')    
    print ('')
    
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:    
    print ('Connected to remote API server')
else:    
    print ('Failed connecting to remote API server')    
    sys.exit('Could not connect to Vrep')

# get the handles of arm joints
err_code, armjoint1_handle = sim.simxGetObjectHandle(clientID,"UR5_joint1", sim.simx_opmode_blocking)
err_code, armjoint2_handle = sim.simxGetObjectHandle(clientID,"UR5_joint2", sim.simx_opmode_blocking)
err_code, armjoint3_handle = sim.simxGetObjectHandle(clientID,"UR5_joint3", sim.simx_opmode_blocking)
err_code, armjoint4_handle = sim.simxGetObjectHandle(clientID,"UR5_joint4", sim.simx_opmode_blocking)
err_code, armjoint5_handle = sim.simxGetObjectHandle(clientID,"UR5_joint5", sim.simx_opmode_blocking)
err_code, armjoint6_handle = sim.simxGetObjectHandle(clientID,"UR5_joint6", sim.simx_opmode_blocking)
# get the handles of hand joints
err_code, endeffector_handle = sim.simxGetObjectHandle(clientID,"suctionPad", sim.simx_opmode_blocking)

# set the arm to position control
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint1_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint2_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint3_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint4_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint5_handle, 2001, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2000, 1, sim.simx_opmode_oneshot)
sim.simxSetObjectIntParameter(clientID, armjoint6_handle, 2001, 1, sim.simx_opmode_oneshot)

# get the collision handles
collision_handle_list = []
for i in range(40):    
    err_code, collision_handle = sim.simxGetCollisionHandle(clientID, "Collision" +str(i), sim.simx_opmode_blocking)
    sim.simxReadCollision(clientID, collision_handle, sim.simx_opmode_streaming)    
    collision_handle_list.append(collision_handle)
    
# You do not need to modify the code above


# function to control the movement of the arm, the input are the angles of joint1, joint2, joint3, joint4, joint5, joint6. The unit are in degrees
def move_arm(armpose):    
    armpose_convert = []    
    for i in range(6):        
        armpose_convert.append(round(armpose[i]/180 * math.pi,3))    
    sim.simxPauseCommunication(clientID,True)    
    sim.simxSetJointTargetPosition(clientID, armjoint1_handle, armpose_convert[0], sim.simx_opmode_oneshot)    
    sim.simxSetJointTargetPosition(clientID, armjoint2_handle, armpose_convert[1], sim.simx_opmode_oneshot)    
    sim.simxSetJointTargetPosition(clientID, armjoint3_handle, armpose_convert[2], sim.simx_opmode_oneshot)    
    sim.simxSetJointTargetPosition(clientID, armjoint4_handle, armpose_convert[3], sim.simx_opmode_oneshot)    
    sim.simxSetJointTargetPosition(clientID, armjoint5_handle, armpose_convert[4], sim.simx_opmode_oneshot)     
    sim.simxSetJointTargetPosition(clientID, armjoint6_handle, armpose_convert[5], sim.simx_opmode_oneshot)        
    sim.simxPauseCommunication(clientID,False)    
    time.sleep(0)
        
# function to check collision
def check_collision():    
    collision_reading = np.zeros(40)    
    is_collision = 0    
    for i in range(40):        
        collision_reading[i] = sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_buffer)[1]        
        if collision_reading[i] == 1:            
            is_collision = 1    
    if is_collision == 1:        
        print('Collision detected!')
        return 1    
    else:        
        return 0

#move_arm([50, 60, 80, 60, -90, 0])
#check_collision()

pose = [0, 0, 0, 0, 0, 0]
f = []
#number 1: set of poses
for i in range(100):
    theta = np.random.randint(-70, 70)
    pose = [theta, theta, theta, theta, theta, theta]
    move_arm(pose)
    check_collision()
    if check_collision() == 0:
        f.append(pose)

for i in f:
    print("Pose " + str(i))
    print()
    
#number 2 neighbors
a = f[10]
b = a
c = a
min = 100
max = 0
for i in f:
    if a[1] == i[1]:
        continue
    if abs(a[1]-i[1]) < min:
        collide = 0
        for j in range(abs(a[1]-i[1])):
            theta = a[1] - j
            pose = [theta, theta, theta, theta, theta, theta]
            move_arm(pose)
            check_collision()
            if check_collision() == 1:
                collide = 1
                break
        if collide == 0:
            b = i
            min = abs(a[1]-i[1])
    
    if abs(a[1] - i[1]) > max:
        collide = 0
        for j in range(abs(a[1]-i[1])):
            theta = a[1] - j
            pose = [theta, theta, theta, theta, theta, theta]
            move_arm(pose)
            check_collision()
            if check_collision() == 1:
                collide = 1
                break
        if collide == 0:
            c = i
            max = abs(a[1]-i[1])
        
    

# no need to modify the code below
# # close the communication between collision handles
for i in range(40):    
    sim.simxReadCollision(clientID, collision_handle_list[i], sim.simx_opmode_discontinue)
    

print("A: " + str(a))
print("B: " + str(b))
print("C: " + str(c))
print ('Program ended')