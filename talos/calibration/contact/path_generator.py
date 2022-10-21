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

# from common_hpp import createGazeConstraints, createGripperLockedJoints, \
#     createLeftArmLockedJoints, \
#     createRightArmLockedJoints, createQuasiStaticEquilibriumConstraint, \
#     createWaistYawConstraint, defaultContext, makeGraph, \
#     makeRobotProblemAndViewerFactory, shrinkJointRange

from agimus_demos.talos.tools_hpp import createGazeConstraints, \
    createGripperLockedJoints, createLeftArmLockedJoints, \
    createRightArmLockedJoints, defaultContext, setGaussianShooter, \
    shrinkJointRange
from common_hpp import createQuasiStaticEquilibriumConstraint, makeGraph, \
    makeRobotProblemAndViewerFactory, Table

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


# constant
MAX_ACC_FREE = 0.25
MAX_SAFETY_FREE = 0.9
MAX_ACC_CONTACT = 0.25
MAX_SAFETY_CONTACT = 0.04

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
    return configurations

# distance between configurations
def distance (ps, q0, q1) :
    ''' Distance between two configurations of the box'''
    assert (len (q0) == ps.robot.getConfigSize ())
    assert (len (q1) == ps.robot.getConfigSize ())
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
        ("SimpleTimeParameterization/maxAcceleration", Any(TC_double, MAX_ACC_FREE))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/safety", Any(TC_double, MAX_SAFETY_FREE))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/order", Any(TC_long, 2))
    paths = pg.generatePathForHandle(handle, q_init)
    # First path is already time parameterized
    # Transform second and third path into PathVector instances to time
    # parameterize them
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/maxAcceleration", Any(TC_double, MAX_ACC_CONTACT))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/safety", Any(TC_double, MAX_SAFETY_CONTACT))
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

def go_to_pre_grasp(ri, pg, handle, qinit, qgoal):
    isp = pg.inStatePlanner
    isp.optimizerTypes = ["EnforceTransitionSemantic",
                                        "SimpleTimeParameterization"]
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/maxAcceleration", Any(TC_double, MAX_ACC_FREE))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/safety", Any(TC_double, MAX_SAFETY_FREE))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/order", Any(TC_long, 2))
    path = pg.generatePathToGoal(handle, qinit, qgoal)
    pg.ps.client.basic.problem.addPath(path)

def pre_grasp_to_contact(ri, pg, gripper, handle, qpg, qg):
    isp = pg.inStatePlanner
    isp.optimizerTypes = ["EnforceTransitionSemantic",
                                        "SimpleTimeParameterization"]
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/maxAcceleration", Any(TC_double, MAX_ACC_FREE))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/safety", Any(TC_double, MAX_SAFETY_FREE))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/order", Any(TC_long, 2))
    paths = pg.generatePathToContact(handle, qpg, qg)
    # First path is already time parameterized
    # Transform second and third path into PathVector instances to time
    # parameterize them
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/maxAcceleration", Any(TC_double, MAX_ACC_CONTACT))
    isp.manipulationProblem.setParameter\
        ("SimpleTimeParameterization/safety", Any(TC_double, MAX_SAFETY_CONTACT))
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

def path_generator(ri, pg, ps, q_init, gripper, handles, iter,
                    create_path=False, write_to_file=False):
    pg.gripper = gripper
    contacts = list()
    pre_grasps = list()
    handle_list = list()
    for handle in handles: 
        count = 0
        while count < iter: 
            res, qpg, qg = pg.generateValidConfigForHandle(handle, q_init, step=3)
            if res:
                pre_grasps.append(qpg)
                contacts.append(qg)
                handle_list.append(handle)
                count += 1

    if create_path:
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
        ordered_cfgs = [q_init] + ordered_cfgs
        for i, idx in enumerate(ordered_idx):
            print(handle_list[idx])
            go_to_pre_grasp(ri, pg, handle_list[idx], ordered_cfgs[i], ordered_cfgs[i+1])
            pre_grasp_to_contact(ri, pg, pg.gripper, handle_list[idx], pre_grasps[idx], contacts[idx])
        go_to_pre_grasp(ri,pg, handle_list[-1], ordered_cfgs[-1], q_init)
        l = 0
        for i in range(ps.numberPaths()):
            l+= ps.pathLength(i)
        print("Path Length: ", l)
    #write to files
    if write_to_file:
        writeConfigsInFile('opt_contacts_{}'.format(handle_list[0]).replace("/","_"), contacts)
        writeConfigsInFile('opt_pregrasps_{}'.format(handle_list[0]).replace("/","_"), pre_grasps)

        with open('opt_handles_{}'.format(handle_list[0]).replace("/","_"), "w") as fw:
            for item in handle_list:
                fw.write("%s\n" % item)

def plan_paths(ri, pg, ps, q_init, gripper, file1, file2, file3):
    pg.gripper = gripper
    contacts = readConfigsInFile(file1)
    pre_grasps = readConfigsInFile(file2)

    with open(file3, "r") as fr:
        handle_list = list()
        for line in fr:
            x = line[:-1]
            handle_list.append(x)

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

    ordered_cfgs = [q_init] + ordered_cfgs

    for i, idx in enumerate(ordered_idx):
        print(handle_list[idx])
        go_to_pre_grasp(ri, pg, handle_list[idx], ordered_cfgs[i], ordered_cfgs[i+1])
        pre_grasp_to_contact(ri, pg, pg.gripper, handle_list[idx], pre_grasps[idx], contacts[idx])
    go_to_pre_grasp(ri,pg, handle_list[-1], ordered_cfgs[-1], q_init)
    l = 0
    for i in range(ps.numberPaths()):
        l+= ps.pathLength(i)
    print("Path Length: ", l)

def check_path_discontinued(ps):
    for i in range(ps.numberPaths()-1):
        q0 = np.array(ps.configAtParam(i, ps.pathLength(i)))
        q1 = np.array(ps.configAtParam(i+1,0))
        assert(np.linalg.norm(q1-q0) < 1e-10), "Path discontinued at the end of {}-th path".format(i)

def erase_all_paths(ps):
    for i in range(ps.numberPaths()):
        ps.erasePath(0)

def mobilize_handles(crobot, dir, dis, handles):
    assert (len(handles) >0)
    for handle in handles:
        _, conf = crobot.getHandlePositionInJoint(handle)
        if dir == 'x':
            conf[0] += dis
        elif dir == 'y':
            conf[1] += dis
        elif dir == 'z':
            conf[2] += dis
        crobot.setHandlePositionInJoint(handle, conf)

def check_in_bound(robot, q):
    out_of_bound = []
    for joint in robot.jointNames:
        if 'root_joint' not in joint:
            rank = robot.rankInConfiguration[joint]
            bounds = robot.getJointBounds(joint)
            assert len(bounds) == 2
            if q[rank] < bounds[0] or q[rank] > bounds[1]:
                print("{} is not in joint bound.".format(joint))
                out_of_bound.append(joint)
    if len(out_of_bound) != 0: 
        print(out_of_bound)
        return False
    else: 
        return True

initConf = [0, 0, 1.02, 0, 0, 0, 1, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
initConf += [.5,0,0,0,0,0,1]
robot, ps, vf, table, objects = makeRobotProblemAndViewerFactory(None)
# set bounds on knee to prevent going too deep
bounds_knee_r = robot.getJointBounds('talos/leg_right_4_joint')
robot.setJointBounds('talos/leg_right_4_joint', [bounds_knee_r[0], bounds_knee_r[1] - 0.5])
bounds_knee_l = robot.getJointBounds('talos/leg_left_4_joint')
robot.setJointBounds('talos/leg_left_4_joint', [bounds_knee_l[0], bounds_knee_l[1] - 0.5])
ri = RosInterface(robot)
tmp = ''
while tmp != 'y':
    print("Are you sure the robot is in half-sitting.")
    tmp = raw_input()

q_init = ri.getCurrentConfig(initConf, 5., 'talos/leg_left_6_joint')

rank_g_left = robot.rankInConfiguration['talos/gripper_left_joint']
rank_g_right = robot.rankInConfiguration['talos/gripper_right_joint']

if q_init[rank_g_left] != 0:
    q_init[rank_g_left] = 0
if q_init[rank_g_right] != 0:
    q_init[rank_g_right] = 0

left_arm_lock  = createLeftArmLockedJoints (ps)
right_arm_lock = createRightArmLockedJoints (ps)
if args.arm == 'left':
    arm_locked = right_arm_lock
elif args.arm == 'right':
    arm_locked = left_arm_lock
else:
    arm_locked = list()

left_gripper_lock, right_gripper_lock = createGripperLockedJoints (ps, q_init)
com_constraint, foot_placement, foot_placement_complement = \
    createQuasiStaticEquilibriumConstraint (ps, q_init)
look_left_hand, look_right_hand = createGazeConstraints(ps)

graph = ConstraintGraph(robot, "graph")
makeGraph(ps, table, graph)

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

res, q, err = graph.generateTargetConfig ("starting_motion", q_init,
                                          q_init)
if not res:
    raise RuntimeError ('Failed to project initial configuration')

from agimus_demos.tools_hpp import PathGenerator
pg = PathGenerator(ps, graph)
ps.setParameter('ConfigurationShooter/Gaussian/standardDeviation', 0.1)
ps.setParameter('ConfigurationShooter/Gaussian/center', q_init)
