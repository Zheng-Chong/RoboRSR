"""Task-space geometry. No hand-authored joint-angle sequence."""
import math
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

def pose(position,rotation=None):
    T=np.eye(4);T[:3,3]=position
    if rotation is not None:T[:3,:3]=rotation
    return T

def smoothstep(t):
    t=float(np.clip(t,0,1));return t*t*t*(10+t*(-15+6*t))

def blend(A,B,t):
    s=smoothstep(t);T=pose((1-s)*A[:3,3]+s*B[:3,3]);T[:3,:3]=Slerp([0,1],Rotation.from_matrix(np.stack([A[:3,:3],B[:3,:3]])))(s).as_matrix();return T

def cup_radius(c,z):
    return c['radius_bottom']+(c['radius_top']-c['radius_bottom'])*z/c['height']

def grasp_transform(spec):
    return pose([0,0,spec['pour']['grasp_height_m']])

def pour_cup_pose(receiver_pose,spec,angle_deg):
    c=spec['objects']['source_cup'];r=spec['objects']['receiver_cup']
    # Keep the low pouring rim above the receiver centre while rotating the cup.
    R=Rotation.from_euler('x',-angle_deg,degrees=True).as_matrix()
    lip=receiver_pose[:3,3]+[0,0,r['height']+spec['pour']['lip_clearance_m']]
    return pose(lip-R@np.array([0,c['radius_top'],c['height']]),R)

def initial_targets(spec):
    source=np.array(spec['objects']['source_cup']['position']);root=np.array(spec['robot']['root_position'])
    return {'right':pose(source+[-.10,0,.22]),'left':pose(root+[.14,.30,.25])}

def plan_targets(spec,source_pose,receiver_pose):
    grasp=grasp_transform(spec);G=source_pose@grasp
    pre=G.copy();pre[:3,3]+=[-.10,0,0]
    lift=G.copy();lift[2,3]+=spec['pour']['lift_m']
    small=G.copy();small[2,3]+=.025
    aligned=pour_cup_pose(receiver_pose,spec,0)@grasp
    tilted=pour_cup_pose(receiver_pose,spec,spec['pour']['max_tilt_deg'])@grasp
    # Durations are minimum-jerk phase durations, not timestamps that bypass feedback.
    return [('pregrasp',pre,3.0),('approach',G,2.5),('close',G,1.5),
            ('test_lift',small,1.8),('lift',lift,2.5),('align',aligned,3.0),
            ('tilt',tilted,6.0),('hold',tilted,3.0),('untilt',aligned,4.0),
            ('return_above',lift,3.0),('lower',G,3.0),('release',G,1.5),('retreat',pre,2.5)]
