# --- PG ---------------------------------------------------------
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix, Inverse_of_matrixHomo, Multiply_of_matrixHomo, Stack_of_vector, PoseRollPitchYawToMatrixHomo, MatrixHomoToPoseRollPitchYaw, Multiply_matrixHomo_vector
from dynamic_graph.sot.dynamics import Dynamic
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.pattern_generator import PatternGenerator,Selector
from dynamic_graph.sot.core.matrix_util import matrixToTuple
# from dynamic_graph.sot.core import FeatureGeneric, FeaturePoint6d, Task, TaskPD
from dynamic_graph.sot.core import FeaturePosture
from dynamic_graph.ros import RosRobotModel


from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)

def initPg(robot,appli):
  # Standard initialization
  appli.pg.parseCmd(":samplingperiod 0.005")
  appli.pg.parseCmd(":previewcontroltime 1.6")
  appli.pg.parseCmd(":walkmode 0")
  appli.pg.parseCmd(":omega 0.0")
  appli.pg.parseCmd(":stepheight 0.05")
  appli.pg.parseCmd(":singlesupporttime 0.780")
  appli.pg.parseCmd(":doublesupporttime 0.020")
  appli.pg.parseCmd(":armparameters 0.5")
  appli.pg.parseCmd(":LimitsFeasibility 0.0")
  appli.pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
  appli.pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
  appli.pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
  if robot.device.name == 'HRP2LAAS' or \
     robot.device.name == 'HRP2JRL':
    appli.pg.parseCmd(":comheight 0.814")
  elif robot.device.name == 'HRP4LIRMM':
    appli.pg.parseCmd(":comheight 0.747")
  else: #default value
    appli.pg.parseCmd(":comheight 0.814")
  appli.pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

  plug(robot.dynamic.position,appli.pg.position)
  plug(appli.com, appli.pg.com)
  plug(robot.dynamic.signal('left-ankle'), appli.pg.leftfootcurrentpos)
  plug(robot.dynamic.signal('right-ankle'), appli.pg.rightfootcurrentpos)
  robotDim = len(robot.dynamic.velocity.value)
  appli.pg.motorcontrol.value = robotDim*(0,)
  appli.pg.zmppreviouscontroller.value = (0,0,0)

  appli.pg.initState()

def addPgToVRMLRobot(robot,appli):
  # Configure Pattern Generator
  modelDir=robot.modelDir+'/'
  robotName=robot.modelName
  specificitiesPath=robot.specificitiesPath
  jointRankPath=robot.jointRankPath
  appli.pg = PatternGenerator('pg')
  appli.pg.setVrmlDir(modelDir+'/')
  appli.pg.setVrml(robotName)
  appli.pg.setXmlSpec(specificitiesPath)
  appli.pg.setXmlRank(jointRankPath)
  # Build Pattern Generator
  appli.pg.buildModel()
  # Initialise Pattern Generator
  initPg(robot,appli)

def addPgToUrdfRobot(robot,appli):
  # Configure Pattern Generator    
  appli.pg = PatternGenerator('pg')
  appli.pg.setUrdfDir(robot.urdfDir)
  appli.pg.setUrdf(robot.urdfName)
  appli.pg.setSoleParameters(robot.ankleLength, robot.ankleWidth)
  if(hasattr(robot, 'jointMap')):
      print "some joints need to be mapped"
      for i in robot.jointMap:
          appli.pg.addJointMapping(i, robot.jointMap[i])
  # Build Pattern Generator
  appli.pg.buildModelUrdf()
  # Initialise Pattern Generator
  initPg(robot,appli)


def addPgTaskToVRMLRobot(robot):
  # --- appli.pg INIT FRAMES ---
  robot.geom = Dynamic("geom")
  print("modelDir: ",robot.modelDir)
  print("modelName:",robot.modelName)
  print("specificitiesPath:",robot.specificitiesPath)
  print("jointRankPath:",robot.jointRankPath)

  robot.geom.setFiles(robot.modelDir, robot.modelName,robot.specificitiesPath,robot.jointRankPath)
  robot.geom.parse()

def addPgTaskToUrdfRobot(robot):
  # --- appli.pg INIT FRAMES ---
  robot.geom = RosRobotModel("geom")
  if(hasattr(robot, 'jointMap')):
      for i in robot.jointMap:
          robot.geom.addJointMapping(i, robot.jointMap[i])
  robot.geom.loadUrdf(robot.urdfDir + robot.urdfName)

def initRobotGeom(robot):
  robot.geom.createOpPoint('rf2','right-ankle')
  robot.geom.createOpPoint('lf2','left-ankle')
  plug(robot.dynamic.position,robot.geom.position)
  robot.geom.ffposition.value = 6*(0,)
  robotDim = len(robot.dynamic.velocity.value)
  robot.geom.velocity.value = robotDim * (0,)
  robot.geom.acceleration.value = robotDim * (0,)

def initZMPRef(robot,appli):
  # --- Selector of Com Ref: when appli.pg is stopped, pg.inprocess becomes 0
  appli.comSelector = Selector('comSelector',['vector', 'ref', appli.com,
                                              appli.pg.comref])
  plug(appli.pg.inprocess,appli.comSelector.selec)

  selecSupportFoot = Selector('selecSupportFoot' \
       ,['matrixHomo','pg_H_sf',appli.pg.rightfootref,appli.pg.leftfootref] \
       ,['matrixHomo','wa_H_sf',robot.geom.rf2,robot.geom.lf2])

  robot.addTrace(appli.pg.name,'rightfootref')
  robot.addTrace(appli.pg.name,'leftfootref')
  robot.addTrace(appli.pg.name,'comref')
  robot.addTrace(appli.pg.name,'zmpref')
  robot.addTrace(appli.pg.name,'inprocess')
  robot.addTrace(robot.device.name,'forceLLEG')
  robot.addTrace(robot.device.name,'forceRLEG')

  plug(appli.pg.SupportFoot,selecSupportFoot.selec)
  sf_H_wa = Inverse_of_matrixHomo('sf_H_wa')
  plug(selecSupportFoot.wa_H_sf,sf_H_wa.sin)
  pg_H_wa = Multiply_of_matrixHomo('pg_H_wa')
  plug(selecSupportFoot.pg_H_sf,pg_H_wa.sin1)
  plug(sf_H_wa.sout,pg_H_wa.sin2)

  # --- Compute the ZMP ref in the Waist reference frame.
  wa_H_pg = Inverse_of_matrixHomo('wa_H_pg')
  plug(pg_H_wa.sout,wa_H_pg.sin)
  wa_zmp = Multiply_matrixHomo_vector('wa_zmp')
  plug(wa_H_pg.sout,wa_zmp.sin1)
  plug(appli.pg.zmpref,wa_zmp.sin2)
  # Connect the ZMPref to OpenHRP in the waist reference frame.
  appli.pg.parseCmd(':SetZMPFrame world')

  robot.addTrace(robot.device.name,'zmp')
  robot.addTrace(pg_H_wa.name,'sout')

def initWaistCoMTasks(robot, appli):
  # ---- TASKS -------------------------------------------------------------------
  # Make sure that the CoM is not controlling the Z
  appli.featureCom.selec.value='011'

  # Build the reference waist pos homo-matrix from PG.

  # Build a left foot roll pitch yaw representation from left foot current pos.
  curLeftPRPY = MatrixHomoToPoseRollPitchYaw('curLeftPRPY')
  plug(robot.dynamic.signal('left-ankle'),curLeftPRPY.sin)
  selecRPYfromCurLeftPRPY = Selec_of_vector('selecRPYfromCurLeftPRPY')
  selecRPYfromCurLeftPRPY.selec(3,6);

  plug(curLeftPRPY.sout,selecRPYfromCurLeftPRPY.sin)

  curRightPRPY = MatrixHomoToPoseRollPitchYaw('curRightPRPY')
  plug(robot.dynamic.signal('right-ankle'),curRightPRPY.sin)
  selecRPYfromCurRightPRPY = Selec_of_vector('selecRPYfromCurRightPRPY')
  selecRPYfromCurRightPRPY.selec(3,6);

  plug(curRightPRPY.sout,selecRPYfromCurRightPRPY.sin)

  addLeftRightRPY = Add_of_vector('addLeftRightRPY')
  plug(selecRPYfromCurLeftPRPY.sout,addLeftRightRPY.sin1)
  plug(selecRPYfromCurLeftPRPY.sout,addLeftRightRPY.sin2)
  
  mulLeftRightRPY = Multiply_double_vector('mulLeftRightRPY')
  mulLeftRightRPY.sin1.value=0.5
  plug(addLeftRightRPY.sout,mulLeftRightRPY.sin2)

  YawFromLeftRightRPY = Multiply_matrix_vector('YawFromLeftRightRPY')
  YawFromLeftRightRPY.sin1.value=matrixToTuple(array([[ 0.,  0.,  0.], \
       [ 0.,  0.,  0.,  ],
       [ 0.,  0.,  1.,  ]]))
  plug(mulLeftRightRPY.sout,YawFromLeftRightRPY.sin2)

  # Build a reference vector from init waist pos and 
  # init left foot roll pitch representation
  waistReferenceVector = Stack_of_vector('waistReferenceVector')
  plug(appli.pg.initwaistposref,waistReferenceVector.sin1)
  #plug(appli.pg.initwaistattref,waistReferenceVector.sin2)
  plug(YawFromLeftRightRPY.sout,waistReferenceVector.sin2)

  waistReferenceVector.selec1(0,3)
  waistReferenceVector.selec2(0,3)
  appli.pg.waistReference=PoseRollPitchYawToMatrixHomo('waistReference')

  robot.addTrace(appli.pg.waistReference.name,'sout')
  robot.addTrace(robot.geom.name,'position')
  robot.addTrace(appli.pg.name,'initwaistposref')
  plug(waistReferenceVector.sout, appli.pg.waistReference.sin)


def initFeetTask(robot,appli):
  robot.selecFeet = Selector('selecFeet',
                             ['matrixHomo','leftfootref', \
                               robot.dynamic.signal('left-ankle'),\
                               appli.pg.leftfootref], \
                             ['matrixHomo','rightfootref', \
                              robot.dynamic.signal('right-ankle'), \
                              appli.pg.rightfootref])

  plug(appli.pg.inprocess,robot.selecFeet.selec)
  appli.tasks['right-ankle'].controlGain.value = 200
  appli.tasks['left-ankle'].controlGain.value = 200

  print "After Task for Right and Left Feet"

def removeDofUsed(jacobian, target):
  for i in range(0,len(jacobian)):
    for j in range(6,len(jacobian[i])):
      if jacobian[i][j] != 0:
        target[j- 6] = False
  return target

def initPostureTask(robot,appli):
  # --- TASK POSTURE --------------------------------------------------
  # set a default position for the joints. 
  appli.features['featurePosition'] = FeaturePosture('featurePosition')
  plug(robot.device.state,appli.features['featurePosition'].state)
  robotDim = len(robot.dynamic.velocity.value)

  appli.features['featurePosition'].posture.value = robot.halfSitting

  # Remove the dofs of the feet.
  postureTaskDofs = [True] * (len(robot.dynamic.position.value) - 6)
  jla = robot.dynamic.signal('Jleft-ankle').value
  postureTaskDofs = removeDofUsed(jla, postureTaskDofs)
  jra = robot.dynamic.signal('Jright-ankle').value
  postureTaskDofs = removeDofUsed(jra, postureTaskDofs)

  for dof,isEnabled in enumerate(postureTaskDofs):
    appli.features['featurePosition'].selectDof(dof+6,isEnabled)
    
  appli.tasks['robot_task_position']=Task('robot_task_position')
  appli.tasks['robot_task_position'].add('featurePosition')

  gainPosition = GainAdaptive('gainPosition')
  gainPosition.set(0.1,0.1,125e3)
  gainPosition.gain.value = 5
  plug(appli.tasks['robot_task_position'].error,gainPosition.error)
  plug(gainPosition.gain,appli.tasks['robot_task_position'].controlGain)
  
def pushTasks(robot,appli):
  # --- TASK COM ---
  plug(appli.pg.dcomref,appli.comdot)
  robot.addTrace (appli.pg.name, 'dcomref')
  plug(appli.comSelector.ref, appli.comRef)

  # --- Plug foot ref ---
  plug(appli.pg.rightfootref,appli.rightAnkle.reference)
  plug(appli.pg.leftfootref,appli.leftAnkle.reference)

  appli.solver.push(appli.tasks['waist'])
  appli.solver.push(appli.tasks['robot_task_position'])
  appli.tasks['com'].controlGain.value = 180

def createGraph(robot,appli):
  initRobotGeom(robot)
  initZMPRef(robot,appli)
  initWaistCoMTasks(robot,appli)

def CreatePG(appli, robot):
  if hasattr(robot, 'urdfName'):
      CreateEverythingForPGwithUrdf(robot,appli)
  else:
      CreateEverythingForPGwithVRML(robot,appli)

def ConnectStandalonePg(appli, robot):
  # Zmp
  plug(appli.pg.zmpref,robot.device.zmp)
  # Waist
  plug(appli.pg.waistReference.sout,appli.waist.reference)
  appli.tasks ['waist'].controlGain.value = 200
  # Controlling also the yaw.
  appli.waist.selec.value = '111100'
  # Feet
  initFeetTask(robot,appli)
  # Posture 
  initPostureTask(robot,appli)
  # Push tasks
  pushTasks(robot,appli)

def CreateEverythingForPG(robot,appli):
  CreatePG(appli, robot)
  ConnectStandalonePg(appli, robot)

def CreateEverythingForPGwithVRML(robot,appli):
  robot.initializeTracer()
  addPgToVRMLRobot(robot,appli)
  addPgTaskToVRMLRobot(robot)
  createGraph(robot,appli)

def CreateEverythingForPGwithUrdf(robot,appli):
  robot.initializeTracer()
  addPgToUrdfRobot(robot,appli)
  addPgTaskToUrdfRobot(robot)
  createGraph(robot,appli)

def walkFewSteps(robot,appli):
  robot.startTracer()
  appli.pg.parseCmd(":stepseq 0.0 0.1025 0.0 0.17 -0.205 0.0 0.17 0.205 0.0 0.17 -0.205 0.0 0.17 0.205 0.0 0.0 -0.205 0.0")

def walkFewStepsCircular(robot,appli):
  robot.startTracer()
  appli.pg.parseCmd(":stepseq 0.0 0.1025 0.0 0.1 -0.205 10.0 0.1 0.205 10.0 0.1 -0.205 10.0 0.1 0.205 10.0 0.0 -0.205 0.0")

def walkAndrei(robot,appli):
  robot.startTracer()
  appli.pg.parseCmd(":SetAlgoForZmpTrajectory Herdt")
  appli.pg.parseCmd(":doublesupporttime 0.1")
  appli.pg.parseCmd(":singlesupporttime 0.7")
  appli.pg.velocitydes.value=(0.01,0.0,0.0)
  appli.pg.parseCmd(":numberstepsbeforestop 4")
  appli.pg.parseCmd(":setVelReference 0.01 0.0 0.0")
  appli.pg.parseCmd(":HerdtOnline")
  if robot.device.name == 'HRP2LAAS' or \
     robot.device.name == 'HRP2JRL':
    appli.pg.parseCmd(":setfeetconstraint XY 0.09 0.06")
  elif robot.device.name == 'HRP4LIRMM':
    appli.pg.parseCmd(":setfeetconstraint XY 0.07 0.06")
  elif robot.device.name == 'ROMEO':
    appli.pg.parseCmd(":setfeetconstraint XY 0.04 0.04")
  else:
    appli.pg.parseCmd(":setfeetconstraint XY 0.02 0.02")
