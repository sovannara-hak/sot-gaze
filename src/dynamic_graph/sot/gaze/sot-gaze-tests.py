#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#

from dynamic_graph.sot.dynamics.tools import *
from optparse import OptionParser

from dynamic_graph.sot.core import FeatureGeneric, FeatureVisualPoint

from dynamic_graph.sot.gaze import Projection2D

from dynamic_graph import plug, enableTrace
import dynamic_graph.sot.core as sotcore
from dynamic_graph.sot.core.robot_simu import RobotSimu
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.matrix_constant import MatrixConstant
from dynamic_graph.sot.core.unary_op import RPYToMatrix
from dynamic_graph.sot.core.derivator import Derivator_of_Vector
from dynamic_graph.sot.dynamics.dynamic_hrp2 import DynamicHrp2
from dynamic_graph.sot.dynamics.angle_estimator import AngleEstimator
from dynamic_graph.sot.dynamics.waist_attitude_from_sensor \
    import WaistPoseFromSensorAndContact
from dynamic_graph.sot.core.feature_point6d import FeaturePoint6d
from dynamic_graph.sot.core.feature_point6d_relative import FeaturePoint6dRelative
from dynamic_graph.sot.core.feature_generic import FeatureGeneric
from dynamic_graph.sot.core.feature_joint_limits import FeatureJointLimits
from dynamic_graph.sot.core.binary_op import Compose_R_and_T
from dynamic_graph.sot.core.task import Task
from dynamic_graph.sot.core.constraint import Constraint
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
from dynamic_graph.sot.core.sot import SOT

try:
    from dynamic_graph.sot.core import OpPointModifior
    OpPointModifier = OpPointModifior
except ImportError:
    from dynamic_graph.sot.core import OpPointModifier

dimension = 36
dyn = DynamicHrp2('dyn')
dyn2 = DynamicHrp2('dyn2')

enableTrace(True, '/home/shak/tmp/sot.out')

#import dynfilessmall
dyn2.setFiles('/home/shak/compil/collodi/share/robot/HRP2JRL/model/',
              'HRP2JRLmainsmall.wrl',
              '/home/shak/compil/collodi/share/hrp2_14/' +
              'HRP2SpecificitiesSmall.xml',
              '/home/shak/compil/collodi/share/hrp2_14/' +
              'HRP2LinkJointRankSmall.xml')
dyn.setFiles('/home/shak/compil/collodi/share/robot/HRP2JRL/model/', 
             'HRP2JRLmainsmall.wrl',                                 
             '/home/shak/compil/collodi/share/hrp2_14/' +            
             'HRP2SpecificitiesSmall.xml',                           
             '/home/shak/compil/collodi/share/hrp2_14/' +            
             'HRP2LinkJointRankSmall.xml')                           
            
dyn2.parse()
zero = VectorConstant('zero')
zero.resize(dimension)

plug(zero.signal('out'), dyn2.signal('position'))
plug(zero.signal('out'), dyn2.signal('velocity'))
plug(zero.signal('out'), dyn2.signal('acceleration'))

dyn2ffposzero = VectorConstant('dyn2ffposzero')

dyn2ffposzero.set((0.,0.,0.,0.,0.,0.))

plug(dyn2ffposzero.signal('out'), dyn2.signal('ffposition'))

dyn2.createOpPoint('0', 'right-wrist')
dyn2.createOpPoint('lh', 'left-wrist')
dyn2.createOpPoint('rleg', 'right-ankle')
dyn2.createOpPoint('lleg', 'left-ankle')
dyn2.createOpPoint('chest', 'chest')

dyn2.setProperty('ComputeVelocity', 'false')
dyn2.setProperty('ComputeCoM', 'false')
dyn2.setProperty('ComputeAccelerationCoM', 'false')
dyn2.setProperty('ComputeMomentum', 'false')
dyn2.setProperty('ComputeZMP', 'false')
dyn2.setProperty('ComputeBackwardDynamics', 'false')

#dyn.signal('gearRatio').value = (0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,207.69,381.54,0.,0.,219.23,231.25,266.67,250.0,145.45,350.0,0.,0.,0.,0.,0.,0.,0.,0.)
#dyn.signal('inertiaRotor').value = (0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,69.6e-7,69.6e-7,0.,0.,69.6e-7,66.0e-7,10.0e-7,66.0e-7,11.0e-7,10.0e-7,0.,0.,0.,0.,0.,0.,0.,0.)

# ----------------------------------------------------
# --- FLEX -------------------------------------------
# ----------------------------------------------------

# ------- Flex based kinematics
flex = AngleEstimator('flex')
flex.setFromSensor(False)

plug(dyn2.signal('lleg'), flex.signal('contactEmbeddedPosition'))
plug(dyn2.signal('chest'), flex.signal('sensorEmbeddedPosition'))

attitudeSensor = RPYToMatrix('attitudeSensor')

# plug attitudeSensor.out flex.sensorWorldRotation
plug(flex.signal('waistWorldPoseRPY'), dyn.signal('ffposition'))

# --- PosFF from leg contact + sensor # DEPRECIATED
posKF = WaistPoseFromSensorAndContact('posKF')
plug(dyn2.signal('lleg'), posKF.signal('contact'))
plug(dyn2.signal('chest'), posKF.signal('position'))
posKF.setFromSensor(True)

# --- DYN With true posFF
dyn.parse()
plug(zero.signal('out'), dyn.signal('velocity'))
plug(zero.signal('out'), dyn.signal('acceleration'))
plug(flex.signal('waistWorldPoseRPY'), dyn.signal('ffposition'))

# ----------------------------------------------------
# --- TASKS ------------------------------------------
# ----------------------------------------------------

# -- TASK Manip
dyn.createOpPoint('0', 'right-wrist')
dyn.createOpPoint('rleg', 'right-ankle')
dyn.createOpPoint('lleg', 'left-ankle')

p6 = FeaturePoint6d('p6')
p6d = FeaturePoint6d('p6d')

comp = Compose_R_and_T('comp')
eye3 = MatrixConstant('eye3')
I3 = ((1., 0., 0.),
      (0., 1., 0.),
      (0., 0., 1.))

comp.displaySignals()
eye3.set(I3)
plug(eye3.signal('out'), comp.signal('sin1'))

t = VectorConstant('t')
# WAIST
t.set((0., 0.095, 0.563816))

# HAND
t.set((0.25, -0.5, .85))
plug(t.signal('out'), comp.signal('sin2'))

plug(comp.signal('sout'), p6d.signal('position'))
plug(dyn.signal('J0'), p6.signal('Jq'))
plug(dyn.signal('0'), p6.signal('position'))
p6.signal('sdes').value = p6d

task = Task('task')
task.add('p6')
gain = GainAdaptive('gain')
gain.setConstant(.2)
plug(task.signal('error'), gain.signal('error'))
plug(gain.signal('gain'), task.signal('controlGain'))

R3 = ((0., 0., -1.),
      (0., 1., 0.),
      (1., 0., 0.))
eye3.set(R3)

p6.signal('selec').value = '000111'
p6.frame('current')

# --- COM
dyn.setProperty('ComputeCoM', 'true')
# dyn.setProperty ComputeVelocity true
# dyn.setProperty ComputeMomentum true
# dyn.setProperty ComputeZMP true

featureCom = FeatureGeneric('featureCom')
plug(dyn.signal('com'), featureCom.signal('errorIN'))
plug(dyn.signal('Jcom'), featureCom.signal('jacobianIN'))
# set featureCom.selec 111

featureComDes = FeatureGeneric('featureComDes')
# set featureComDes.errorIN [2](0,-0)
featureCom.signal('sdes').value = featureComDes

taskCom = Task('taskCom')
taskCom.add('featureCom')

taskCom.signal('controlGain').value = .3
# set taskCom.controlGain 1

# --- CHEST
# --- Task Chest for blocking the chest joints
# featureChest = FeatureGeneric('featureChest')
# featureChest.signal('jacobianIN').value = ((0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.),(0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.))

# featureChest.signal('errorIN').value =(0.,0.)
# taskChest = Task('taskChest')
# taskChest.add('featureChest')
# taskChest.signal('controlGain').value = 1.



# --- Task RightArm for blocking the right arm
featureRightArm = FeatureGeneric('featureRightArm')
featureRightArm.signal('jacobianIN').value = reduce(lambda jac, i : jac + ((22+i)*(0.,)+(1.,)+(13-i)*(0.,),), range(7), ())
featureRightArm.signal('errorIN').value = (0.,0.,0.,0.,0.,0.,0.)
taskRightArm = Task('taskRightArm')
taskRightArm.add('featureRightArm')
taskRightArm.signal('controlGain').value = 1.

# --- Task LeftArm for  blocking the left arm
featureLeftArm = FeatureGeneric('featureLeftArm')
featureLeftArm.signal('jacobianIN').value = reduce(lambda jac, i : jac + ((29+i)*(0.,)+(1.,)+(6-i)*(0.,),), range(7), ())
featureLeftArm.signal('errorIN').value = 7*(0,)
taskLeftArm = Task('taskLeftArm')
taskLeftArm.add('featureLeftArm')
taskLeftArm.signal('controlGain').value = 1

# --- Joint Limits
featureJl = FeatureJointLimits('featureJl')
featureJl.actuate()
plug(dyn.signal('position'), featureJl.signal('joint'))
plug(dyn.signal('upperJl'), featureJl.signal('upperJl'))
plug(dyn.signal('lowerJl'), featureJl.signal('lowerJl'))
taskJl = Task('taskJl')
taskJl.add('featureJl')
taskJl.signal('controlGain').value = .1

# --- LEGS
featureLegs = FeatureGeneric('featureLegs')
jacobianLegs = MatrixConstant('jacobianLegs')
jac = 12*[dimension*[0.]]
jac[0][6] = 1.
jac[1][7] = 1.
jac[2][8] = 1.
jac[3][9] = 1.
jac[4][10] = 1.
jac[5][11] = 1.
jac[6][12] = 1.
jac[7][13] = 1.
jac[8][14] = 1.
jac[9][15] = 1.
jac[10][16] = 1.
jac[11][17] = 1.
jac = tuple(map(tuple, jac))
jacobianLegs.set(jac)
plug(jacobianLegs.signal('out'), featureLegs.signal('jacobianIN'))

vectorLegs = VectorConstant('vectorLegs')
vectorLegs.set(7*(0.,))
plug(vectorLegs.signal('out'), featureLegs.signal('errorIN'))
# set featureLegs.errorIN [12](0,0,0,0,0,0,0,0,0,0,0,0)
taskLegs = Task('taskLegs')
taskLegs.add('featureLegs')
taskLegs.signal('controlGain').value = 1.

# --- TWOFEET
featureTwofeet = FeaturePoint6dRelative('featureTwofeet')
plug(dyn.signal('Jrleg'),  featureTwofeet.signal('Jq'))
plug(dyn.signal('Jlleg'),  featureTwofeet.signal('JqRef'))
plug(dyn.signal('rleg'),  featureTwofeet.signal('position'))
plug(dyn.signal('lleg'), featureTwofeet.signal('positionRef'))

taskTwofeet = Task('taskTwofeet')
taskTwofeet.add('featureTwofeet')
taskTwofeet.signal('controlGain').value = 0.

# --- CONTACT CONSTRAINT
legs = Constraint('legs')
legs.addJacobian('dyn.Jlleg')

# --- SOT
sot = SOT('sot')
sot.signal('damping').value = 1e-6
sot.addConstraint('legs')
sot.setNumberDofs(dimension)

# plug dyn.inertia sot.weight
# sot.push taskTwofeet
# sot.push taskLegs
sot.push('task')
sot.push('taskCom')
# sot.push taskChest
# sot.gradient taskJl

featureComDes.signal('errorIN').value = (0.004,-0.09,.6)

zeroCom = VectorConstant('zeroCom')
zeroCom.set(dimension*(0.,))


dyn.createOpPoint('Waist', 'waist')

OpenHRP = RobotSimu('OpenHRP')
OpenHRP.set((0.,0.,0.,0.,0.,0.,0.,0.,-1.04720,2.09440,-1.04720,0.,0.,0.,-1.04720,2.09440,-1.04720,0.,0.0000,0.,-0.,-0.,0.17453,-0.17453,-0.17453,-0.87266,0.,-0.,0.1,0.,0.17453,-0.17453,-0.17453,-0.87266,0.,-0.,0.1,0.))

plug(sot.signal('control'), OpenHRP.signal('control'))
plug(OpenHRP.signal('state'), dyn.signal('position'))
plug(OpenHRP.signal('state'), dyn2.signal('position'))
plug(OpenHRP.signal('attitude'), posKF.signal('attitudeIN'))
plug(OpenHRP.signal('attitude'), flex.signal('sensorWorldRotation'))
eye = []
for i in xrange(36):
    tmp = []
    for j in xrange(36):
        if i != j:
            tmp.append(0.)
        else:
            tmp.append(1.)
    eye.append(tuple(tmp))


proj = Projection2D('project')
proj.displaySignals()
proj.help()

vis = FeatureVisualPoint('vis')
visDes = FeatureVisualPoint('visDes')

dyn.displaySignals()

headGaze = OpPointModifier('headGaze')

OpenHRP.displaySignals()

#new Compose<R+T> headOffset

#new MatrixConstant mheadOffset
#mheadOffset.resize 3 3 
#mheadOffset.eye

#new VectorConstant theadOffset
#theadOffset.resize 3
#theadOffset.fill 0.0
#theadOffset.[] 2 0.25

#plug mheadOffset.out headOffset.in1
#plug theadOffset.out headOffset.in2

#compute headOffset.out
#headGaze.transfoSignal headOffset.out

#plug dyn.Jhead headGaze.jacobianIN
#plug dyn.head headGaze.positionIN

#new VectorConstant pointxyz
#pointxyz.resize 3
#pointxyz.[] 0 0
#pointxyz.[] 1 -0.4
#pointxyz.[] 2 1.6

#new VectorConstant errorVis
#errorVis.resize 2
#errorVis.fill 0

#plug errorVis.out visDes.xy

#new projection2D proj
#plug pointxyz.out proj.point
#plug headGaze.position proj.transform

#plug headGaze.jacobian vis.Jq
#plug proj.image vis.xy
#set vis.sdes visDes

#new Task gaze
#gaze.add vis
#set gaze.controlGain 1.4
