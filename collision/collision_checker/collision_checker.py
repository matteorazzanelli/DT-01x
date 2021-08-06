import itertools
import random
from typing import List
import numpy as np
from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)
from dt_protocols.collision_protocol import Primitive

__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(Wcoll=self.params.environment, robot_body=self.params.body, robot_pose=data.pose)
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(Wcoll: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose) -> bool:
    robot = robot_body[0]
    robot.pose = robot_pose
    for obs in Wcoll:
        if collision(obs,robot):
            return True
    return False

def collision(obs: PlacedPrimitive, robot: PlacedPrimitive) -> bool:

    # rototranslate the robot
    p1 = polygon_from_placed_primitive(robot.pose, robot.primitive)

    if isinstance(obs.primitive, Rectangle):
        # rototranslate the obstacle
        p2 = polygon_from_placed_primitive(obs.pose, obs.primitive)
        return checkRectRect(p1, p2)

    if isinstance(obs.primitive, Circle):
        return checkRectCircle(robot, p1, obs)

    print('################################################################### ERRORE ########################################################')

##########################################################
##########################################################

def checkRectCircle(robot: PlacedPrimitive, p1: List, obs: PlacedPrimitive):
    dist = ((obs.pose.x - robot.pose.x)**2 + (obs.pose.y - robot.pose.y)**2)**(1/2)
    dist_a = ((p1[0][0] - robot.pose.x)**2 + (p1[0][1] - robot.pose.y)**2)**(1/2)
    dist_b = ((p1[1][0] - robot.pose.x)**2 + (p1[1][1] - robot.pose.y)**2)**(1/2)
    dist_c = ((p1[2][0] - robot.pose.x)**2 + (p1[2][1] - robot.pose.y)**2)**(1/2)
    dist_d = ((p1[3][0] - robot.pose.x)**2 + (p1[3][1] - robot.pose.y)**2)**(1/2)

    w = ((p1[0][0] - p1[1][0])**2 + (p1[0][1] - p1[1][1])**2)**0.5
    h = ((p1[0][0] - p1[3][0])**2 + (p1[0][1] - p1[3][1])**2)**0.5
    width = w if w<h else h
    height = h if w<h else w

    if dist < dist_a or dist < dist_b or dist < dist_c or dist < dist_d or dist < width or dist < height or dist < obs.primitive.radius:
        return True

    O_a = ((p1[0][0] - obs.pose.x)**2 + (p1[0][1] - obs.pose.y)**2)**(1/2)
    O_b = ((p1[1][0] - obs.pose.x)**2 + (p1[1][1] - obs.pose.y)**2)**(1/2)
    O_c = ((p1[2][0] - obs.pose.x)**2 + (p1[2][1] - obs.pose.y)**2)**(1/2)
    O_d = ((p1[3][0] - obs.pose.x)**2 + (p1[3][1] - obs.pose.y)**2)**(1/2)

    if O_a < obs.primitive.radius or O_b < obs.primitive.radius or O_c < obs.primitive.radius or O_d < obs.primitive.radius:
        return True
    
    return False

##########################################################
##########################################################

def checkRectRect(p1, p2):

    '''
    Separating Axis Theorem
    '''
    V1 = [np.array(v, 'float64') for v in p1]
    V2 = [np.array(v, 'float64') for v in p2]

    E = []
    for i in range(len(V1)):
        pre = i
        post = (i + 1) % len(V1)
        edge = [-(V1[post][1] - V1[pre][1]) , V1[post][0] - V1[pre][0]]
        E.append(edge)
    for i in range(len(V2)):
        pre = i
        post = (i + 1) % len(V2)
        edge = [-(V2[post][1] - V2[pre][1]) , V2[post][0] - V2[pre][0]]
        E.append(edge)

    for e in E:
        aMaxProj = -np.inf
        aMinProj = np.inf
        bMaxProj = -np.inf
        bMinProj = np.inf
        # We are only interested in max/min projections
        for v in V1:
            proj = v[0] * e[0] + v[1] * e[1]
            if (proj < aMinProj): aMinProj = proj
            if (proj > aMaxProj): aMaxProj = proj
        for v in V2:
            proj = v[0] * e[0] + v[1] * e[1]
            if (proj < bMinProj): bMinProj = proj
            if (proj > bMaxProj): bMaxProj = proj
        # check if intervals overlap
        if (aMaxProj < bMinProj or aMinProj > bMaxProj): return False
    # no separating axis found: the polygons intersect.
    return True

def polygon_from_placed_primitive(pose: FriendlyPose, primitive: Primitive):
    robot_rad = np.deg2rad(pose.theta_deg)
    T_o_r =  np.array([ [np.cos(robot_rad),-np.sin(robot_rad),pose.x],
                        [np.sin(robot_rad),np.cos(robot_rad),pose.y],
                        [0, 0, 1]])
    T_r_a = np.array([[1, 0, primitive.xmin],[0, 1, primitive.ymax],[0, 0, 1]])
    T_r_b = np.array([[1, 0, primitive.xmax],[0, 1, primitive.ymax],[0, 0, 1]])
    T_r_c = np.array([[1, 0, primitive.xmax],[0, 1, primitive.ymin],[0, 0, 1]])
    T_r_d = np.array([[1, 0, primitive.xmin],[0, 1, primitive.ymin],[0, 0, 1]])
    p = []
    p.append(np.dot(T_o_r,T_r_a)[0:2,2])
    p.append(np.dot(T_o_r,T_r_b)[0:2,2])
    p.append(np.dot(T_o_r,T_r_c)[0:2,2])
    p.append(np.dot(T_o_r,T_r_d)[0:2,2])

    return p