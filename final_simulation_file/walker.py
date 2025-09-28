from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import numpy as np

client = RemoteAPIClient()
sim = client.require('sim')
simIK = client.require('simIK')

DEG2RAD = math.pi / 180.0


class SysCall:
    def __init__(self):
        self.ikEnv = simIK.createEnvironment()    
        self.legs = {}

        # Base handles
        self.ikBase_handle = sim.getObjectHandle('/arnold_base')
        self.antBase = sim.getObjectHandle('/arnold_base/Base_target')

        # Gait parameters
        self.stepProgression = 0.0
        self.stepVelocity = 0.8
        self.stepAmplitude = 0.16
        self.stepHeight = 0.04

        # Motion control
        self.realMS = 0
        self.mS = 1
        self.overcome = False

        # Wheels & switches
        self.wheelVel = 2
        self.legTips = [-1] * 4
        self.legTargets = [-1] * 4
        self.legJoints = [-1] * 4
        self.wheels = [-1] * 4
        self.switches = [-1] * 4
        self.initialPos = [None] * 4

        # Diagonal trot phases
        self.phase_offsets = {
            "front_left": 3,
            "back_right": 3,
            "front_right": 1,
            "back_left": 1,
        }

        # Leg configs
        leg_configs = {
            "front_left": {
                "joints": ["/arnold_base/link3","/arnold_base/link3/link3_respondable/link3_2"],
                "tip": "/arnold_base/link3/link3_respondable/link3_2/link3_2_respondable/tip1",
                "target": "/arnold_base/Body_target/target1",
                "switch": "/arnold_base/link3/link3_respondable/link3_2/link3_2_respondable/link_3_2_3",
                "wheel": "/arnold_base/link3/link3_respondable/link3_2/link3_2_respondable/link_3_2_3/link_3_2_3_respondable/wheel_3",
            },
            "front_right": {
                "joints": ["/arnold_base/link4","/arnold_base/link4/link4_respondable/link4_2"],
                "tip": "/arnold_base/link4/link4_respondable/link4_2/link4_2_respondable/tip2",
                "target": "/arnold_base/Body_target/target2",
                "switch": "/arnold_base/link4/link4_respondable/link4_2/link4_2_respondable/link_4_2_3",
                "wheel": "/arnold_base/link4/link4_respondable/link4_2/link4_2_respondable/link_4_2_3/link_4_2_3_respondable/wheel_4",
            },
            "back_right": {
                "joints": ["/arnold_base/link1","/arnold_base/link1/link1_respondable/link1_2"],
                "tip": "/arnold_base/link1/link1_respondable/link1_2/link1_2_respondable/tip3",
                "target": "/arnold_base/Body_target/target3",
                "switch": "/arnold_base/link1/link1_respondable/link1_2/link1_2_respondable/link_1_2_3",
                "wheel": "/arnold_base/link1/link1_respondable/link1_2/link1_2_respondable/link_1_2_3/link_1_2_3_respondable/wheel_1",
            },
            "back_left": {
                "joints": ["/arnold_base/link2","/arnold_base/link2/link2_respondable/link2_2"],
                "tip": "/arnold_base/link2/link2_respondable/link2_2/link2_2_respondable/tip4",
                "target": "/arnold_base/Body_target/target4",
                "switch": "/arnold_base/link2/link2_respondable/link2_2/link2_2_respondable/link_2_2_3",
                "wheel": "/arnold_base/link2/link2_respondable/link2_2/link2_2_respondable/link_2_2_3/link_2_2_3_respondable/wheel_2",
            },
        }

        # Initialize legs
        for i, (name, cfg) in enumerate(leg_configs.items()):
            self.legs[name] = self._init_leg(cfg)
            self.legTips[i] = sim.getObjectHandle(cfg["tip"])
            self.legTargets[i] = sim.getObjectHandle(cfg["target"])
            self.wheels[i] = sim.getObjectHandle(cfg["wheel"])
            self.switches[i] = sim.getObjectHandle(cfg["switch"])
            self.legJoints[i] = [sim.getObjectHandle(j) for j in cfg["joints"]]
            self.initialPos[i] = sim.getObjectPosition(self.legTips[i], self.antBase)

    def _init_leg(self, cfg):
        simBase = sim.getObject('/arnold_base')
        simTip = sim.getObject(cfg["tip"])
        simTarget = sim.getObject(cfg["target"])

        # Use damped least squares everywhere for stability
        ikGroup = simIK.createGroup(self.ikEnv)
        simIK.setGroupCalculation(self.ikEnv, ikGroup, simIK.method_damped_least_squares, 1, 99)
        simIK.addElementFromScene(self.ikEnv, ikGroup, simBase, simTip, simTarget, simIK.constraint_pose)

        return {"ikGroup": ikGroup}

    def footTrajectory(self, sp):
        A = self.stepAmplitude
        H = self.stepHeight

        if sp < 0.5:  # stance
            x = A/2 - (sp / 0.5) * A
            z = 0.0
        else:        # swing
            t = (sp - 0.5) / 0.5
            theta = t * math.pi
            R = A / 2.0
            x = -R * math.cos(theta)
            z = H * math.sin(theta)

        return x, z

    def motorSpeed(self):
        sim.setJointTargetVelocity(self.wheels[0], self.wheelVel)
        sim.setJointTargetVelocity(self.wheels[1], -self.wheelVel)
        sim.setJointTargetVelocity(self.wheels[2], -self.wheelVel)
        sim.setJointTargetVelocity(self.wheels[3], self.wheelVel)

    def wheelSwitch(self, angle):
        rad = angle * DEG2RAD
        sim.setJointTargetPosition(self.switches[0], -rad)    
        sim.setJointTargetPosition(self.switches[1], rad) 
        sim.setJointTargetPosition(self.switches[2], rad) 
        sim.setJointTargetPosition(self.switches[3], -rad) 

    def walk(self, dt):
        for i, (leg, data) in enumerate(self.legs.items()):
            phase = self.phase_offsets[leg]
            sp = (self.stepProgression + (phase - 1)/4) % 1.0

            x, z = self.footTrajectory(sp)
            offset = [x * self.realMS, 0, z * self.realMS]
            p = [self.initialPos[i][0] + offset[0],
                 self.initialPos[i][1] + offset[1],
                 self.initialPos[i][2] + offset[2]]

            sim.setObjectPosition(self.legTargets[i], self.antBase, p)
            simIK.handleGroup(self.ikEnv, data["ikGroup"], {'syncWorlds': True})

        self.stepProgression += dt * self.stepVelocity

    def sysCall_actuation(self):
        dt = sim.getSimulationTimeStep()
        t = sim.getSimulationTime()
        
        # Smooth step size
        dx = self.mS - self.realMS
        if abs(dx) > dt*0.1:
            dx = dx * dt * 0.5 / abs(dx)
        self.realMS += dx
        
        self.walk(dt)


# --- Run simulation ---
call = SysCall()
sim.setStepping(True)
sim.startSimulation()

while sim.getSimulationState() != sim.simulation_stopped:
    call.sysCall_actuation()
    sim.step()

sim.stopSimulation()