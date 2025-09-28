import vrep
import time
import cv2
import numpy as np
import math
import sys

def angle(clientID, motor_1, motor_2, motor_3, motor_4, angle_1, servo_1, servo_2, servo_3, servo_4):
    r = vrep.simxSetJointTargetPosition(clientID, motor_1, -1*(angle_1)*math.pi/180, vrep.simx_opmode_streaming)
    r = vrep.simxSetJointTargetPosition(clientID, motor_2, 1*(angle_1)*math.pi/180, vrep.simx_opmode_streaming)
    r = vrep.simxSetJointTargetPosition(clientID, motor_3, 1*(angle_1)*math.pi/180, vrep.simx_opmode_streaming)
    r = vrep.simxSetJointTargetPosition(clientID, motor_4, -1*(angle_1)*math.pi/180, vrep.simx_opmode_streaming)

    r = vrep.simxSetJointTargetPosition(clientID, servo_1, (angle_1)*2*math.pi/180, vrep.simx_opmode_streaming)
    r = vrep.simxSetJointTargetPosition(clientID, servo_2, -2*(angle_1)*math.pi/180, vrep.simx_opmode_streaming)
    r = vrep.simxSetJointTargetPosition(clientID, servo_3, -2*(angle_1)*math.pi/180, vrep.simx_opmode_streaming)
    r = vrep.simxSetJointTargetPosition(clientID, servo_4, (angle_1)*2*math.pi/180, vrep.simx_opmode_streaming)
    
def speedMotor(clientID, omni_1, omni_2, omni_3, omni_4, velr, vell):
    errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_1,velr, vrep.simx_opmode_streaming)    
    errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_2,velr, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_3,vell, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_4,vell, vrep.simx_opmode_streaming)

PI = math.pi

vrep.simxFinish(-1)

clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID!=-1:
    print('Connected to remote API server')
else:
    print("Failed to connect to remote API Server")
    sys.exit("Couldn't Connect")
    
ultrasonic = []
res, v0 = vrep.simxGetObjectHandle(clientID, 'v0', vrep.simx_opmode_oneshot_wait)
res, v1 = vrep.simxGetObjectHandle(clientID, 'v1', vrep.simx_opmode_oneshot_wait)

res, motor_1 = vrep.simxGetObjectHandle(clientID, 'Motor_1', vrep.simx_opmode_oneshot_wait)
res, motor_2 = vrep.simxGetObjectHandle(clientID, 'Motor_2', vrep.simx_opmode_oneshot_wait)
res, motor_3 = vrep.simxGetObjectHandle(clientID, 'Motor_3', vrep.simx_opmode_oneshot_wait)
res, motor_4 = vrep.simxGetObjectHandle(clientID, 'Motor_4', vrep.simx_opmode_oneshot_wait)

res, servo_1 = vrep.simxGetObjectHandle(clientID, 'Servo_1', vrep.simx_opmode_oneshot_wait)
res, servo_2 = vrep.simxGetObjectHandle(clientID, 'Servo_2', vrep.simx_opmode_oneshot_wait)
res, servo_3 = vrep.simxGetObjectHandle(clientID, 'Servo_3', vrep.simx_opmode_oneshot_wait)
res, servo_4 = vrep.simxGetObjectHandle(clientID, 'Servo_4', vrep.simx_opmode_oneshot_wait)

res, omni_1 = vrep.simxGetObjectHandle(clientID, 'Omni_1', vrep.simx_opmode_oneshot_wait)
res, omni_2 = vrep.simxGetObjectHandle(clientID, 'Omni_2', vrep.simx_opmode_oneshot_wait)
res, omni_3 = vrep.simxGetObjectHandle(clientID, 'Omni_3', vrep.simx_opmode_oneshot_wait)
res, omni_4 = vrep.simxGetObjectHandle(clientID, 'Omni_4', vrep.simx_opmode_oneshot_wait)

r = vrep.simxSetJointTargetPosition(clientID, motor_1, -1*(0)*math.pi/180, vrep.simx_opmode_streaming)
r = vrep.simxSetJointTargetPosition(clientID, motor_2, 1*(0)*math.pi/180, vrep.simx_opmode_streaming)
r = vrep.simxSetJointTargetPosition(clientID, motor_3, 1*(0)*math.pi/180, vrep.simx_opmode_streaming)
r = vrep.simxSetJointTargetPosition(clientID, motor_4, -1*(0)*math.pi/180, vrep.simx_opmode_streaming)

r = vrep.simxSetJointTargetPosition(clientID, servo_1, (0)*2*math.pi/180, vrep.simx_opmode_streaming)
r = vrep.simxSetJointTargetPosition(clientID, servo_2, -2*(0)*math.pi/180, vrep.simx_opmode_streaming)
r = vrep.simxSetJointTargetPosition(clientID, servo_3, -2*(0)*math.pi/180, vrep.simx_opmode_streaming)
r = vrep.simxSetJointTargetPosition(clientID, servo_4, (0)*2*math.pi/180, vrep.simx_opmode_streaming)

##err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_streaming)
##time.sleep(1)
##while (vrep.simxGetConnectionId(clientID) != -1):
##    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v0, 0, vrep.simx_opmode_buffer)
##    if err == vrep.simx_return_ok:
##        img = np.array(image, dtype = np.uint8)
##        img.resize([resolution[0], resolution[1], 3])
##        img2 = img.ravel()
##        vrep.simxSetVisionSensorImage(clientID, v1, img2, 0, vrep.simx_opmode_oneshot)
##    elif err == vrep.simx_return_novalue_flag:
##      print("no image yet")
##      pass
##    else:
##      print(err)

sensor_h=[]
sensor_val=np.array([]) 

for x in range(1,4+1):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'sensor'+str(x),vrep.simx_opmode_oneshot_wait)
        sensor_h.append(sensor_handle)        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint))

t = time.time()
a1 = 0
posCollapsed = False
countpos = 0
counter = 0
levelRaised = False
sensor_loc=np.array([-PI/2, -PI/2, -PI/2]) 
while (time.time()-t)<1000:
    sensor_val=np.array([])    
    detected = []
    for x in range(1,4+1):
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h[x-1],vrep.simx_opmode_buffer)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) 
        detected.append(detectionState)
    
    sensor_sq=sensor_val[0:3]*sensor_val[0:3]
    print(sensor_sq)
    min_ind=np.where(sensor_sq==np.min(sensor_sq))
    min_ind=min_ind[0][0]

    if sensor_sq[min_ind] < 0.2 and detected[0] == True:
        print("Raising "+str(a1))
        if a1 == 90:
            print("Turning")
            for angle_1 in range(a1, 0, -15):
                angle(clientID, motor_1, motor_2, motor_3, motor_4, angle_1, servo_1, servo_2, servo_3, servo_4)
                time.sleep(0.75)
                a1 -= 15
            if a1<0:
                a1 = 0
            c = time.time()
            while(time.time()-c <2.85):
                velr = -10
                vell = -10
                speedMotor(clientID, omni_1, omni_2, omni_3, omni_4, velr, vell)

            c = time.time()
            while (time.time()-c < 2.85):
                velr = 10
                vell = 20
                speedMotor(clientID, omni_1, omni_2, omni_3, omni_4, velr, vell)

        if a1<0:
            a1 = 0
        velr = 0
        vell = 0
        speedMotor(clientID, omni_1, omni_2, omni_3, omni_4, velr, vell)
        for angle_1 in range(a1, 91, 15):
            angle(clientID, motor_1, motor_2, motor_3, motor_4, angle_1, servo_1, servo_2, servo_3, servo_4)
            time.sleep(0.75)
            levelRaised = True
            if sensor_val[0] < 0.5 :
                print("Still blocking")
                a1 += 15
                break            
    else:
        if detected[3] == False:
            counter +=1
        if detected[3] == True and levelRaised:
            countpos += 1 

        velr = 10
        vell = 10
        speedMotor(clientID, omni_1, omni_2, omni_3, omni_4, velr, vell)
        
        if a1>20 and levelRaised == True and countpos > 0 and detected[3] == False:
            print("False 3 and collapsing")
            velr = 0
            vell = 0
            speedMotor(clientID, omni_1, omni_2, omni_3, omni_4, velr, vell)
            
            time.sleep(0.75)
            for angle_1 in range(a1, -1, -15):
                angle(clientID, motor_1, motor_2, motor_3, motor_4, angle_1, servo_1, servo_2, servo_3, servo_4)
                time.sleep(0.75)
                posCollapsed = True
                a1 -= 15
            levelRaised = False      

##    if detected[0] == False and detected[1] == False and detected[2] == False:
##        errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_1,10, vrep.simx_opmode_streaming)    
##        errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_2,10, vrep.simx_opmode_streaming)
##        errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_3,10, vrep.simx_opmode_streaming)
##        errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_4,10, vrep.simx_opmode_streaming)


##    if detected[1] == True and detected[0] == True:
##        errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_1,10, vrep.simx_opmode_streaming)    
##        errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_2,10, vrep.simx_opmode_streaming)
##        errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_3,10, vrep.simx_opmode_streaming)
##        errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_4,10, vrep.simx_opmode_streaming)
##        
##    if detected[1] == True and sensor_sq[min_ind] < 0.5:
##        c = time.time()
##        while (time.time()-c <2.5):
##            errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_1,10, vrep.simx_opmode_streaming)    
##            errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_2,10, vrep.simx_opmode_streaming)
##            errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_3,20, vrep.simx_opmode_streaming)
##            errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_4,20, vrep.simx_opmode_streaming)
##
##    if detected[2] == True and sensor_sq[min_ind] < 0.5:
##        c = time.time()
##        while (time.time()-c <2.5):
##            errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_1,20, vrep.simx_opmode_streaming)    
##            errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_2,20, vrep.simx_opmode_streaming)
##            errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_3,10, vrep.simx_opmode_streaming)
##            errorCode=vrep.simxSetJointTargetVelocity(clientID,omni_4,10, vrep.simx_opmode_streaming)                
