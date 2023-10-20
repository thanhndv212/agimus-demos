# Copyright 2020 CNRS - Airbus SAS
# Author: Florent Lamiraux, Joseph Mirabel, Alexis Nicolin
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This script selectively does one of the following
#
#  1. generate n configurations where the camera looks at the left wrist
#     options --N=n --arm=left,
#  2. generate n configurations where the camera looks at the right wrist
#     options --N=n --arm=right,
#  3. reads configurations from file './data/all-configurations.csv')
#
#  Then, it computes a path going through all these configurations.

from csv import reader, writer
import argparse, numpy as np
from CORBA import Any, TC_double, TC_long
from hpp import Transform
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Constraints, ConstraintGraph, \
    ProblemSolver, Rule
from hpp.corbaserver.manipulation.robot import CorbaClient, HumanoidRobot
from hpp.gepetto.manipulation import ViewerFactory
from hpp.corbaserver.manipulation.constraint_graph_factory import \
    ConstraintGraphFactory
from agimus_demos.tools_hpp import RosInterface, concatenatePaths
from hpp_idl.hpp import Error as HppError

from common_hpp import createGazeConstraints, createGripperLockedJoints, \
    createLeftArmLockedJoints, \
    createRightArmLockedJoints, createQuasiStaticEquilibriumConstraint, \
    createWaistYawConstraint, defaultContext, makeGraph, \
    makeRobotProblemAndViewerFactory, shrinkJointRange

from hpp.gepetto import PathPlayer

loadServerPlugin (defaultContext, "manipulation-corba.so")

client = CorbaClient(context=defaultContext)
client.manipulation.problem.resetProblem()

# parse arguments
p = argparse.ArgumentParser (description=
                             'Procuce motion to calibrate arms and camera')
p.add_argument ('--arm', type=str, metavar='arm',
                default=None,
                help="which arm: 'right' or 'left'")
p.add_argument ('--N', type=int, metavar='N', default=0,
                help="number of configurations generated")
args = p.parse_args ()

# Write configurations in a file in CSV format
def writeConfigsInFile (filename, configs):
    with open (filename, "w") as f:
        w = writer (f)
        for q in configs:
            w.writerow (q)

# Read configurations in a file in CSV format
def readConfigsInFile (filename):
    with open(filename, "r") as f:
        configurations = list()
        r = reader (f)
        for line in r:
            configurations.append(map(float,line))
    return configurationsps.concatenatePath(0,1)
ps.erasePath(1)
    distance = ps.hppcorba.problem.getDistance ()
    return distance.call (q0, q1)

# Check that target frame of gaze constraint is not behind the camera
def validateGazeConstraint (ps, q, whichArm):
    robot = ps.robot
    robot.setCurrentConfig (q)
    Mcamera = Transform(robot.getLinkPosition
                        (ps.robot.camera_frame))
    Mtarget = Transform(robot.getLinkPosition("talos/arm_" + whichArm +
                                              "_7_link"))
    z = (Mcamera.inverse () * Mtarget).translation [2]
    if z < .1: return False
    return True
    
def shootRandomConfigs (ps, graph, q0, n):
    robot = ps.robot
    configs = list ()
    i = 0
    while i < n:
        q = robot.shootRandomConfig ()          
        res, q1, err = graph.generateTargetConfig ("Loop | f", q0, q)
        if not res: continue
        res = validateGazeConstraint (ps, q1, args.arm)
        if not res: continue
        res, msg = robot.isConfigValid (q1)
        if res:
            configs.append (q1)
            i += 1
    return configs

def buildDistanceMatrix (ps, configs):
    N = len (configs)
    # Build matrix of distances between box poses
    dist = np.matrix (np.zeros (N*N).reshape (N,N))
    for i in range (N):
        for j in range (i+1,N):
            dist [i,j] = distance (ps, configs [i], configs [j])
            dist [j,i] = dist [i,j]
    return dist

def orderConfigurations (ps, configs):
    N = len (configs)
    # Order configurations according to naive solution to traveler
    # salesman problem
    notVisited = range (1,N)
    visited = range (0,1)
    dist = buildDistanceMatrix (ps, configs)
    while len (notVisited) > 0:
        # rank of current configuration in visited
        i = visited [-1]
        # find closest not visited configuration
        m = 1e20
        closest = None
        for j in notVisited:
            if dist [i,j] < m:
                m = dist [i,j]
                closest = j
        notVisited.remove (closest)
        visited.append (closest)
    orderedConfigs = list ()
    for i in visited:
        orderedConfigs.append (configs [i])
    return orderedConfigs

# get indices of closest configs to config [i]
def getClosest (dist,i,n):
    d = list()
    for j in range (dist.shape[1]):
        if j!=i:
            d.append ((j,dist[i,j]))
    d.sort (key=lambda x:x[1])
    return zip (*d)[0][:n]

def buildRoadmap (configs):
    if len(configs)==0: return
    dist = buildDistanceMatrix (ps, configs)
    for q in configs:
        ps.addConfigToRoadmap(q)
    for i, q in enumerate (configs):
        ps.addConfigToRoadmap (q)
        closest = getClosest (dist,i,20)
        for j in closest:
            if dist[i,j] != 0 and j>i:
                qi=configs[i]
                qj=configs[j]
                res, pid, msg = ps.directPath(qi,qj,True)
                if res:
                    ps.addEdgeToRoadmap (qi,qj,pid,True)
    # clear paths
    for i in range(ps.numberPaths(),0,-1):
        ps.erasePath (i-1)

def visitConfigurations (ps, configs):
    nOptimizers = len(ps.getSelected("PathOptimizer"))
    for q_init, q_goal in zip (configs, configs [1:]):
        ps.resetGoalConfigs ()
        ps.setInitialConfig (q_init)
        ps.addGoalConfig (q_goal)
        ps.solve ()
        for i in range(nOptimizers):
            # remove non optimized paths
            pid = ps.numberPaths () - 2
            ps.erasePath (pid)

def goToContact(ri, pg, gripper, handle, q_init):
    pg.gripper = gripper
    q_init = ri.getCurrentConfig(q_init, 5., 'talos/leg_left_6_joint')
    res, q_init, err = pg.graph.generateTargetConfig('starting_motion', q_init,
                                                     q_init)
    if not res:
        raise RuntimeError('Failed to project initial configuration')
    isp = pg.inStatePlanner
    isp.optimizerTypes = ["EnforceTransitionSemantic",
                                        "SimpleTimeParameterization"]
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/maxAcceleration", Any(TC_double, 0.1))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/safety", Any(TC_double, 0.5))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/order", Any(TC_long, 2))
    paths = pg.generatePathForHandle(handle, q_init)
    # First path is already time parameterized
    # Transform second and third path into PathVector instances to time
    # parameterize them
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/maxAcceleration", Any(TC_double, 0.01))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/safety", Any(TC_double, 0.02))
    finalPaths = [paths[0],]
    for i, p in enumerate(paths[1:]):
        path = p.asVector()
        for optType in isp.optimizerTypes:
            optimizer = isp.wd(isp.ps.hppcorba.problem.createPathOptimizer\
                (optType, isp.manipulationProblem))
            optpath = optimizer.optimize(path)
            # optimize can return path if it couldn't find a better one.
            # In this case, we have too different client refering to the
            # same servant.
            # thus the following code deletes the old client, which
            # triggers deletion of the servant and the new path points to
            # nothing...
            # path = pg.wd(optimizer.optimize(path))
            from hpp.corbaserver.tools import equals
            if not equals(path, optpath):
                path = isp.wd(optpath)
        finalPaths.append(path)
    pg.ps.client.basic.problem.addPath(finalPaths[0])
    pg.ps.client.basic.problem.addPath(concatenatePaths(finalPaths[1:]))

def pre_grasp_to_contact(ri, pg, gripper, handle, qpg, qg):
    isp = pg.inStatePlanner
    isp.optimizerTypes = ["EnforceTransitionSemantic",
                                        "SimpleTimeParameterization"]
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/maxAcceleration", Any(TC_double, 0.1))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/safety", Any(TC_double, 0.5))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/order", Any(TC_long, 2))
    paths = pg.generatePathToContact(handle, qpg, qg)
    # First path is already time parameterized
    # Transform second and third path into PathVector instances to time
    # parameterize them
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/maxAcceleration", Any(TC_double, 0.01))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/safety", Any(TC_double, 0.02))
    finalPaths = []
    for i, p in enumerate(paths[0:]):
        path = p.asVector()
        for optType in isp.optimizerTypes:
            optimizer = isp.wd(isp.ps.hppcorba.problem.createPathOptimizer\
                (optType, isp.manipulationProblem))
            optpath = optimizer.optimize(path)
            # optimize can return path if it couldn't find a better one.
            # In this case, we have too different client refering to the
            # same servant.
            # thus the following code deletes the old client, which
            # triggers deletion of the servant and the new path points to
            # nothing...
            # path = pg.wd(optimizer.optimize(path))
            from hpp.corbaserver.tools import equals
            if not equals(path, optpath):
                path = isp.wd(optpath)
        finalPaths.append(path)
    # pg.ps.client.basic.problem.addPath(finalPaths[0])
    pg.ps.client.basic.problem.addPath(concatenatePaths(finalPaths[0:]))

initConf = [0, 0, 1.02, 0, 0, 0, 1, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0]

robot, ps, vf, table, objects = makeRobotProblemAndViewerFactory(None)
initConf += [.5,0,0,0,0,0,1]
ri = RosInterface(robot)

left_arm_lock  = createLeftArmLockedJoints (ps)
right_arm_lock = createRightArmLockedJoints (ps)
if args.arm == 'left':
    arm_locked = right_arm_lock
elif args.arm == 'right':
    arm_locked = left_arm_lock
else:
    arm_locked = list()

left_gripper_lock, right_gripper_lock = createGripperLockedJoints (ps, initConf)
com_constraint, foot_placement, foot_placement_complement = \
    createQuasiStaticEquilibriumConstraint (ps, initConf)
look_left_hand, look_right_hand = createGazeConstraints(ps)

graph = makeGraph(ps, table)

# Add other locked joints in the edges.
for edgename, edgeid in graph.edges.items():
    if edgename[:7] == "Loop | " and edgename[7] != 'f':
        graph.addConstraints(
            edge=edgename, constraints=Constraints(numConstraints=\
                                                   ['table/root_joint',])
        )
# Add gaze and and equilibrium constraints and locked grippers to each node of
# the graph
prefixLeft = 'talos/left_gripper'
prefixRight = 'talos/right_gripper'
l = len(prefixLeft)
r = len(prefixRight)
for nodename, nodeid in graph.nodes.items():
    graph.addConstraints(
        node=nodename, constraints=Constraints(numConstraints=\
            com_constraint + foot_placement + left_gripper_lock + \
            right_gripper_lock
            )
        )
    if nodename[:l] == prefixLeft:
        graph.addConstraints(
            node=nodename, constraints=Constraints(numConstraints=\
                                                   [look_left_hand,]))
    if nodename[:r] == prefixRight:
        graph.addConstraints(
            node=nodename, constraints=Constraints(numConstraints=\
                                                   [look_right_hand,]))

# add foot placement complement in each edge.
for edgename, edgeid in graph.edges.items():
    graph.addConstraints(
        edge=edgename,
        constraints=Constraints(numConstraints=foot_placement_complement),
    )

# On the real robot, the initial configuration as measured by sensors is very
# likely not in any state of the graph. State "starting_state" and transition
# "starting_motion" are aimed at coping with this issue.
graph.createNode("starting_state")
graph.createEdge("starting_state", "free", "starting_motion", isInNode="starting_state")
graph.createEdge(
    "free",
    "starting_state",
    "go_to_starting_state",
    isInNode="starting_state",
    weight=0,
)
graph.addConstraints(
    edge="starting_motion",
    constraints=Constraints(numConstraints=['table/root_joint',]),)
graph.addConstraints(
    edge="go_to_starting_state",
    constraints=Constraints(numConstraints=['table/root_joint',]),
)
graph.initialize ()

ps.setParameter("SimpleTimeParameterization/safety", 0.2)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", .1)
# ps.addPathOptimizer ("RandomShortcut")
ps.addPathOptimizer ("EnforceTransitionSemantic")
ps.addPathOptimizer ("SimpleTimeParameterization")

res, q, err = graph.generateTargetConfig ("starting_motion", initConf,
                                          initConf)
if not res:
    raise RuntimeError ('Failed to project initial configuration')

from agimus_demos.tools_hpp import PathGenerator
pg = PathGenerator(ps, graph)
pg.gripper = 'talos/left_gripper'

ps.setParameter('ConfigurationShooter/Gaussian/standardDeviation', 0.1)
ps.setParameter('ConfigurationShooter/Gaussian/center', initConf)

# contacts = list()
# pre_grasps = list()
# handle_list = list()
# for handle in table.handles: 
#     count = 0
#     while count < 3: 
#         res, qpg, qg = pg.generateValidConfigForHandle(handle, initConf, step=3)
#         if res:
#             pre_grasps.append(qpg)
#             contacts.append(qg)
#             handle_list.append(handle)
#             count += 1

contacts = readConfigsInFile('27_opt_contacts')
pre_grasps = readConfigsInFile('27_opt_pregrasps')

with open('27_opt_handles', "r") as fr:
    handle_list = list()
    for line in fr:
        x = line[:-1]
        handle_list.append(x)


v = vf.createViewer()

# create a list of paths linking ordered visting pre_grasps
ordered_cfgs = orderConfigurations(ps, pre_grasps)

# reorder contacts 
ordered_contacts = list()
ordered_idx = list()
ordered_handle_list = list()
for cfg in ordered_cfgs:
    if cfg in pre_grasps:
        idx = pre_grasps.index(cfg)
        ordered_idx.append(idx)
        ordered_contacts.append(contacts[idx])
        ordered_handle_list.append(handle_list[idx])
ordered_cfgs = ordered_cfgs + [initConf]

# buildRoadmap(ordered_cfgs)

# number of optimizer
nOptimizers = len(ps.getSelected("PathOptimizer"))

# res, pid, msg = ps.directPath(initConf, ordered_cfgs[0],True)
ps.setInitialConfig(initConf)
ps.addGoalConfig(ordered_cfgs[0])
ps.solve()

# # erase first 2 configs
for j in range(nOptimizers): ps.erasePath(ps.numberPaths() - 1)

for i, idx in enumerate(ordered_idx):
    print(handle_list[idx])
    pre_grasp_to_contact(ri, pg, pg.gripper, handle_list[idx], pre_grasps[idx], contacts[idx])
    
    # pre_grasp to contact 
    ps.resetGoalConfigs()
    ps.setInitialConfig(ordered_cfgs[i])
    ps.addGoalConfig(ordered_cfgs[i+1])
    ps.solve()

    # erase first 2 configs 
    for j in range(nOptimizers): ps.erasePath(ps.numberPaths() - 1)

# concatenate all paths in one path
while ps.numberPaths() >1: 
            print("Pregrasp configuration found :", qpg)
            print("Goal configuration found     :", qg)
    ps.concatenatePath(0,1) 
    ps.erasePath(1)

print(ps.numberPaths())
pp = PathPlayer(v)

  def generatePathToContact(self, handle, qpg, qg, NrandomConfig=10,
                              step=3):
        edge = self.gripper + " > " + handle
        ok = False
        for nTrial in range(NrandomConfig):
            # from pregrasp to grasp
            self.inStatePlanner.setEdge(edge + " | f_12")
            res, p2, msg = self.inStatePlanner.directPath(qpg, qg, True)
            if not res: p2 = None
            if not p2: continue
            # back to pregrasp
            p3 = self.wd(p2.reverse())
            return [ p2, p3]
        raise RuntimeError('failed fo compute a path.')


<robot name="table">
  <contact name="top">
    <link name="base_link"/>
    <point>
      +0.25 +0.30 0.75
      +0.25 -0.30 0.75
      -0.25 -0.30 0.75
      -0.25 +0.30 0.75
    </point>
    <shape>
      4 3 2 1 0
    </shape>
  </contact>
  <handle name="contact_01" clearance="0.05">
    <position xyz="0.0 0 0.76" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>
</robot>


# writeConfigsInFile('27_opt_contacts', contacts)
# writeConfigsInFile('27_opt_pregrasps', pre_grasps)

# with open('27_opt_handles', "w") as fw:
#     for item in handle_list:
#         fw.write("%s\n" % item)


<robot name="table">
  <contact name="top">
    <link name="base_link"/>
    <point>
      +0.4 +0.20 0.68
      +0.4 -1.00 0.68
      -0.4 -1.00 0.68
      -0.4 +0.20 0.68
    </point>
    <shape>
      4 3 2 1 0
    </shape>
  </contact>

  <contact name="side">
    <link name="base_link"/>
    <point>
      +0.4 +0.20 0.68
      +0.4 +0.20 0.00
      -0.4 +0.20 0.68
      -0.4 +0.20 0.00
    </point>
    <shape>
      4 3 2 1 0
    </shape>
  </contact>
  
  <handle name="contact_01" clearance="0.05">
    <position xyz="0.32 -0.23 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_02" clearance="0.05">
    <position xyz="0.32 -0.4 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_03" clearance="0.05">
    <position xyz="0.32 -0.57 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_04" clearance="0.05">
    <position xyz="0.15 -0.23 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_05" clearance="0.05">
    <position xyz="0.15 -0.4 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_06" clearance="0.05">
    <position xyz="0.15 -0.57 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_07" clearance="0.05">
    <position xyz="-0.02 -0.23 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_08" clearance="0.05">
    <position xyz="-0.02 -0.4 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_09" clearance="0.05">
    <position xyz="-0.02 -0.57 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>
</robot>


# robot.py 
    ## Set a given position to a handle in joint frame
    def setHandlePositionInJoint (self, handleName, position):
        self.client.manipulation.robot.setHandlePositionInJoint (handleName, position)
# robot.impl.cc
      void Robot::setHandlePositionInJoint (const char* handleName,
          const ::hpp::Transform_ position)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
          HandlePtr_t handle = robot->handles.get (handleName);
          if (!handle)
            throw Error ("This handle does not exists.");
          Transform3f t;
          hppTransformToTransform3f (position, t);
          handle->localPosition(t);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
        }
      }

# robot.impl.hh
          virtual void setHandlePositionInJoint (const char* handleName,
              const ::hpp::Transform_ position);

# handle.cc
void Handle::localPosition(const Transform3f& localPosition) {
  localPosition_ = localPosition;
}
# handle.hh
 /// Set local position in joint frame
  void localPosition(const Transform3f& localPosition);





# left arm on side, right arm on top 
<robot name="table">
  <link name="base_link">
    <visual>
      <origin xyz="0.15 -0.6 .67"/>
      <geometry>
        <box size=".80 1.20 .02"/>
      </geometry>
    </visual>
    <visual>
      <origin xyz="0.2125 -1.19 .33"/>
      <geometry>
        <box size=".675 .02 .66"/>
      </geometry>
    </visual>
    <visual>
      <origin xyz="0.2125 -0.01 .33"/>
      <geometry>
        <box size=".675 .02 .66"/>
      </geometry>
    </visual>
    <visual>
      <origin xyz="0.54 -0.6 .33"/>
      <geometry>
        <box size=".02 1.16 .66"/>
      </geometry>
    </visual>

    <collision>
      <origin xyz="0.15 -0.6 .67"/>
      <geometry>
        <box size=".80 1.20 .02"/>
      </geometry>
    </collision>
    <collision>
      <origin xyz="0.2125 -1.19 .33"/>
      <geometry>
        <box size=".675 .02 .66"/>
      </geometry>
    </collision>
    <collision>
      <origin xyz="0.2125 -0.01 .33"/>
      <geometry>
        <box size=".675 .02 .66"/>
      </geometry>
    </collision>
    <collision>
      <origin xyz="0.54 -0.6 .33"/>
      <geometry>
        <box size=".02 1.16 .66"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.2"/>
      <origin xyz="0.15 -0.6 0.65" rpy="0 0 0"/>
      <inertia ixx="0.1"  ixy="0"  ixz="0" iyy="0.1" iyz="0" izz="0.1" />
    </inertial>
  </link>
</robot>



<robot name="table">
  <contact name="top">
    <link name="base_link"/>
    <point>
      +0.7 0.6 0.68
      +0.7 -0.6 0.68
      -0.1 -0.6 0.68
      -0.1 0.6 0.68
    </point>
    <shape>
      4 3 2 1 0
    </shape>
  </contact>

  <contact name="side">
    <link name="base_link"/>
    <point>
      +0.525 0.6 0.68
      +0.525 0.6 0.00
      -0.1 0.6 0.68
      -0.1 0.6 0.00
    </point>
    <shape>
      4 3 2 1 0
    </shape>
  </contact>

  <contact name="front">
    <link name="base_link"/>
    <point>
      +0.6 0.6 0.68
      +0.6 -0.6 0.00
      +0.6 0.6 0.68
      +0.6 -0.6 0.00
    </point>
    <shape>
      4 3 2 1 0
    </shape>
  </contact>
  
  <handle name="contact_01" clearance="0.05">
    <position xyz="0.25 -0.24 0.6" xyzw="0.0 0.0 -0.7071068 0.7071068"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_02" clearance="0.05">
    <position xyz="0.1 -0.24 0.6" xyzw="0.0 0.0 -0.7071068 0.7071068"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_03" clearance="0.05">
    <position xyz="-0.03 -0.24 0.6" xyzw="0.0 0.0 -0.7071068 0.7071068"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_04" clearance="0.05">
    <position xyz="0.25 -0.24 0.5" xyzw="0.0 0.0 -0.7071068 0.7071068"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_05" clearance="0.05">
    <position xyz="0.1 -0.24 0.5" xyzw="0.0 0.0 -0.7071068 0.7071068"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_06" clearance="0.05">
    <position xyz="-0.03 -0.24 0.5" xyzw="0.0 0.0 -0.7071068 0.7071068"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_07" clearance="0.05">
    <position xyz="0.25 -0.24 0.4" xyzw="0.0 0.0 -0.7071068 0.7071068"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_08" clearance="0.05">
    <position xyz="0.1 -0.24 0.4" xyzw="0.0 0.0 -0.7071068 0.7071068"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_09" clearance="0.05">
    <position xyz="-0.03 -0.24 0.4" xyzw="0.0 0.0 -0.7071068 0.7071068"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_10" clearance="0.05">
    <position xyz="0.25 -0.3 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_11" clearance="0.05">
    <position xyz="0.25 -0.4 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_12" clearance="0.05">
    <position xyz="0.25 -0.5 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_13" clearance="0.05">
    <position xyz="0.1 -0.3 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_14" clearance="0.05">
    <position xyz="0.1 -0.4 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_15" clearance="0.05">
    <position xyz="0.1 -0.5 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_16" clearance="0.05">
    <position xyz="-0.03 -0.3 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_17" clearance="0.05">
    <position xyz="-0.03 -0.4 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_18" clearance="0.05">
    <position xyz="-0.03 -0.5 0.67" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_19" clearance="0.05">
    <position xyz="-0.09 -0.3 0.6" xyzw="0.0 0.0 0.0 1.0"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_20" clearance="0.05">
    <position xyz="-0.09 -0.4 0.6" xyzw="0.0 0.0 0.0 1.0"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_21" clearance="0.05">
    <position xyz="-0.09 -0.5 0.6" xyzw="0.0 0.0 0.0 1.0"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_22" clearance="0.05">
    <position xyz="-0.09 -0.3 0.5" xyzw="0.0 0.0 0.0 1.0"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_23" clearance="0.05">
    <position xyz="-0.09 -0.4 0.5" xyzw="0.0 0.0 0.0 1.0"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_24" clearance="0.05">
    <position xyz="-0.09 -0.5 0.5" xyzw="0.0 0.0 0.0 1.0"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_25" clearance="0.05">
    <position xyz="-0.09 -0.3 0.4" xyzw="0.0 0.0 0.0 1.0"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_26" clearance="0.05">
    <position xyz="-0.09 -0.4 0.4" xyzw="0.0 0.0 0.0 1.0"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_27" clearance="0.05">
    <position xyz="-0.09 -0.5 0.4" xyzw="0.0 0.0 0.0 1.0"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>
</robot>

<robot name="table">
  <contact name="top">
    <link name="base_link"/>
    <point>
      +0.7 0.6 0.68
      +0.7 -0.6 0.68
      -0.1 -0.6 0.68
      -0.1 0.6 0.68
    </point>
    <shape>
      4 3 2 1 0
    </shape>
  </contact>

  <contact name="side">
    <link name="base_link"/>
    <point>
      +0.7 0.6 0.68
      +0.7 0.6 0.00
      0.025 0.6 0.68
      0.025 0.6 0.00
    </point>
    <shape>
      4 3 2 1 0
    </shape>
  </contact>

  <contact name="front">
    <link name="base_link"/>
    <point>
      +0.8 0.6 0.68
      +0.8 0.6 0.00
      +0.8 -0.6 0.68
      +0.8 -0.6 0.00
    </point>
    <shape>
      4 3 2 1 0
    </shape>
  </contact>
  
  <handle name="contact_01" clearance="0.07">
    <position xyz="0.5 0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_02" clearance="0.07">
    <position xyz="0.5 0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_03" clearance="0.07">
    <position xyz="0.5 0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_04" clearance="0.07">
    <position xyz="0.5 -0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_05" clearance="0.07">
    <position xyz="0.5 -0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_06" clearance="0.07">
    <position xyz="0.5 -0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

    <handle name="contact_07" clearance="0.07">
    <position xyz="0.4 0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_08" clearance="0.07">
    <position xyz="0.4 0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_09" clearance="0.07">
    <position xyz="0.4 0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_10" clearance="0.07">
    <position xyz="0.4 -0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_11" clearance="0.07">
    <position xyz="0.4 -0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_12" clearance="0.07">
    <position xyz="0.4 -0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_13" clearance="0.07">
    <position xyz="0.3 0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_14" clearance="0.07">
    <position xyz="0.3 0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_15" clearance="0.07">
    <position xyz="0.3 0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_16" clearance="0.07">
    <position xyz="0.3 -0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_17" clearance="0.07">
    <position xyz="0.3 -0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_18" clearance="0.07">
    <position xyz="0.3 -0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_19" clearance="0.07">
    <position xyz="0.2 0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_20" clearance="0.07">
    <position xyz="0.2 0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_21" clearance="0.07">
    <position xyz="0.2 0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_22" clearance="0.07">
    <position xyz="0.2 -0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_23" clearance="0.07">
    <position xyz="0.2 -0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_24" clearance="0.07">
    <position xyz="0.2 -0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_25" clearance="0.07">
    <position xyz="0.1 0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_26" clearance="0.07">
    <position xyz="0.1 0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_27" clearance="0.07">
    <position xyz="0.1 0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_28" clearance="0.07">
    <position xyz="0.1 -0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_29" clearance="0.07">
    <position xyz="0.1 -0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_30" clearance="0.07">
    <position xyz="0.1 -0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_31" clearance="0.07">
    <position xyz="0.0 0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_32" clearance="0.07">
    <position xyz="0.0 0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_33" clearance="0.07">
    <position xyz="0.0 0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_34" clearance="0.07">
    <position xyz="0.0 -0.1 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_35" clearance="0.07">
    <position xyz="0.0 -0.3 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

  <handle name="contact_36" clearance="0.07">
    <position xyz="0.0 -0.5 0.675" xyzw="-0.5 0.5 0.5 0.5"/>
    <link name="base_link"/>
    <mask> 1 1 1 0 1 1 </mask>
  </handle>

</robot>
