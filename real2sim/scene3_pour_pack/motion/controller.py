"""Feedback-gated pouring with Isaac Lab's PhysX-Jacobian differential IK.

No object teleportation, cup attachment, baked joint playback or fake liquid
transfer is used. A failed grasp or a stalled reach stops the sequence.
"""
from pathlib import Path
import json,math
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from isaaclab.controllers import DifferentialIKController,DifferentialIKControllerCfg
from isaaclab.utils.math import quat_apply,subtract_frame_transforms,matrix_from_quat,skew_symmetric_matrix
from task_geometry import pose,blend,smoothstep,grasp_transform,pour_cup_pose,initial_targets,cup_radius

def from_state(v):
    v=v.detach().cpu().numpy() if isinstance(v,torch.Tensor) else np.asarray(v)
    T=pose(v[:3]);T[:3,:3]=Rotation.from_quat(v[[4,5,6,3]]).as_matrix();return T

def pose_tensor(T,device):
    q=Rotation.from_matrix(T[:3,:3]).as_quat()
    return torch.tensor([[*T[:3,3],q[3],*q[:3]]],dtype=torch.float32,device=device)

class ArmServo:
    def __init__(self,robot,side,offset,dt):
        self.robot=robot;self.side=side;self.dt=dt
        names=[side+'_'+n+'_joint' for n in ['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']]
        self.ids=[robot.joint_names.index(n) for n in names];self.body=robot.body_names.index(side+'_dex1_base_link')
        if not robot.is_fixed_base:raise RuntimeError('This pouring controller requires the delivered fixed-base G1 model.')
        self.jac_idx=self.body-1;self.offset=torch.tensor([offset],dtype=torch.float32,device=robot.device)
        self.ik=DifferentialIKController(DifferentialIKControllerCfg(command_type='pose',use_relative_mode=False,ik_method='dls'),num_envs=1,device=robot.device)
        self.command=robot.data.joint_pos[:,self.ids].clone();self.velocity=torch.zeros_like(self.command)

    def actual(self):
        b=self.robot.data.body_state_w[:,self.body,:7]
        p=b[:,:3]+quat_apply(b[:,3:7],self.offset)
        return from_state(torch.cat([p,b[:,3:7]],dim=1)[0])

    def step(self,target):
        r=self.robot;b=r.data.body_state_w[:,self.body,:7];root=r.data.root_state_w[:,:7]
        shift=quat_apply(b[:,3:7],self.offset);p=b[:,:3]+shift
        J=r.root_physx_view.get_jacobians()[:,self.jac_idx,:,:][:,:,self.ids].clone()
        J[:,:3,:]-=torch.bmm(skew_symmetric_matrix(shift),J[:,3:,:])
        Rinv=matrix_from_quat(root[:,3:7]).transpose(1,2)
        J[:,:3,:]=torch.bmm(Rinv,J[:,:3,:]);J[:,3:,:]=torch.bmm(Rinv,J[:,3:,:])
        actual_p,actual_q=subtract_frame_transforms(root[:,:3],root[:,3:7],p,b[:,3:7])
        goal=pose_tensor(target,r.device);goal_p,goal_q=subtract_frame_transforms(root[:,:3],root[:,3:7],goal[:,:3],goal[:,3:7])
        self.ik.set_command(torch.cat([goal_p,goal_q],dim=-1))
        raw=self.ik.compute(actual_p,actual_q,J,r.data.joint_pos[:,self.ids])
        limits=r.data.soft_joint_pos_limits[:,self.ids,:]
        raw=torch.clamp(raw,limits[:,:,0]+.005,limits[:,:,1]-.005)
        desired_velocity=torch.clamp((raw-self.command)/self.dt,-.65,.65)
        self.velocity+=torch.clamp(desired_velocity-self.velocity,-1.8*self.dt,1.8*self.dt)
        self.command+=self.velocity*self.dt
        self.command=torch.clamp(self.command,limits[:,:,0]+.005,limits[:,:,1]-.005)
        r.set_joint_position_target(self.command,joint_ids=self.ids)
        r.set_joint_velocity_target(self.velocity,joint_ids=self.ids)
        actual=self.actual();pe=np.linalg.norm(actual[:3,3]-target[:3,3]);ae=np.linalg.norm(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).as_rotvec())
        return float(pe),float(ae)

class PourController:
    phases=[('settle',1.5),('pregrasp',3.),('approach',2.5),('close',1.5),('test_lift',1.8),('lift',2.5),
            ('align',3.0),('tilt',6.),('hold',3.),('untilt',4.),('return_above',3.),('lower',3.),('release',1.5),('retreat',2.5)]
    def __init__(self,robot,source,receiver,water,spec,dt,log_path):
        self.robot=robot;self.source=source;self.receiver=receiver;self.water=water;self.s=spec;self.dt=dt
        self.servos={side:ArmServo(robot,side,spec['pour']['tcp_in_dex1_base_m'],dt) for side in ['left','right']}
        self.left_target=initial_targets(spec)['left'];self.target=self.servos['right'].actual();self.start=self.target.copy()
        self.source_start=from_state(source.data.root_state_w[0,:7]);self.grasp=grasp_transform(spec)
        self.phase_idx=0;self.phase='settle';self.phase_time=0.;self.phase_elapsed=0.;self.total=0.;self.stable=0.
        self.done=False;self.failed=False;self.reason='';self.closed=False;self.tilt=0.;self.stop_tilt=spec['pour']['max_tilt_deg']
        self.log=open(log_path,'w',encoding='utf-8');self.ticks=0;self.measurement={};self.last_error=(0.,0.)
        self.finger_ids=[robot.joint_names.index(f'right_dex1_finger_joint_{i}') for i in [1,2]]
        c=spec['objects']['source_cup'];self.fit=(2*cup_radius(c,spec['pour']['grasp_height_m'])-spec['pour']['neutral_finger_gap_m'])/2
        self.finger_command=spec['pour']['open_finger_m']
        print('[POUR] settle: waiting for actual contacts and fluid readback.',flush=True)

    def source_pose(self):return from_state(self.source.data.root_state_w[0,:7])
    def receiver_pose(self):return from_state(self.receiver.data.root_state_w[0,:7])

    def phase_goal(self):
        base=self.source_start@self.grasp
        if self.phase=='settle':return self.start
        if self.phase in ['pregrasp','retreat']:
            T=base.copy();T[0,3]-=.10;return T
        if self.phase in ['approach','close','lower','release']:return base
        if self.phase=='test_lift':
            T=base.copy();T[2,3]+=.025;return T
        if self.phase in ['lift','return_above']:
            T=base.copy();T[2,3]+=self.s['pour']['lift_m'];return T
        if self.phase in ['align','untilt']:return pour_cup_pose(self.receiver_pose(),self.s,0)@self.grasp
        return pour_cup_pose(self.receiver_pose(),self.s,self.stop_tilt)@self.grasp

    def fail(self,reason):
        if self.failed:return
        self.failed=True;self.reason=reason;self.done=True
        self.write_log('FAILED',reason);print('[POUR] STOPPED:',reason,flush=True)

    def write_log(self,status,reason=''):
        self.log.write(json.dumps({'time_s':round(self.total,4),'phase':self.phase,'status':status,'reason':reason,
            'source_pose':self.source_pose().tolist(),'receiver_pose':self.receiver_pose().tolist(),
            'tcp_error_m':self.last_error[0],'orientation_error_deg':math.degrees(self.last_error[1]),
            'tilt_command_deg':self.tilt,**self.measurement})+'\n');self.log.flush()

    def obstacle_guard(self):
        # Runtime proximity guard. This is not a global collision-free path planner.
        for side in ['left','right']:
          for name,radius in [('elbow_link',.035),('wrist_roll_link',.026),('dex1_base_link',.030)]:
            p=self.robot.data.body_state_w[0,self.robot.body_names.index(side+'_'+name),:3].cpu().numpy()
            if abs(p[0])<.30+radius and abs(p[1])<.60+radius and p[2]<.75+radius+.003:
                return 'Robot link approached the tabletop: '+side+'_'+name
            for n,c in self.s['objects'].items():
                if not n.startswith('basket'):continue
                low=np.array(c['position'])+[-c['size'][0]/2,-c['size'][1]/2,0]-radius
                high=np.array(c['position'])+np.array(c['size'])*[.5,.5,1]+radius
                if np.all(p>low)&np.all(p<high):return 'Robot link approached '+n
        return None

    def step(self):
        if self.done:return
        self.total+=self.dt;self.phase_elapsed+=self.dt;self.ticks+=1
        if self.ticks%12==0:
            if self.water:self.measurement=self.water.measure(self.source_pose(),self.receiver_pose())
            guard=self.obstacle_guard()
            if guard:self.fail(guard);return
            if self.source_pose()[2,3]<.70:self.fail('Source cup dropped below tabletop.');return
            if self.receiver_pose()[2,2]<.95:self.fail('Receiver cup tipped over.');return
            if self.water and self.measurement['spilled_ml']>self.water.ml*self.s['pour']['max_spill_fraction']:
                self.fail('Spilled-water threshold exceeded.');return
            if self.phase in ['lift','align','tilt','hold','untilt','return_above']:
                expected=self.servos['right'].actual()@np.linalg.inv(self.grasp);actual=self.source_pose()
                if np.linalg.norm(expected[:3,3]-actual[:3,3])>.035:
                    self.fail('Cup slipped relative to gripper.');return
        if self.phase_elapsed>max(12,3*self.phases[self.phase_idx][1]):self.fail('Phase timeout: '+self.phase);return
        duration=self.phases[self.phase_idx][1]
        # Pause progression while the arm catches up with the moving Cartesian goal.
        if self.last_error[0]<.020 and self.last_error[1]<.15:self.phase_time+=self.dt
        a=min(1.,self.phase_time/duration)
        if self.phase=='tilt':
            self.tilt=self.s['pour']['max_tilt_deg']*smoothstep(a)
            self.target=pour_cup_pose(self.receiver_pose(),self.s,self.tilt)@self.grasp
        elif self.phase=='untilt':
            self.tilt=self.stop_tilt*(1-smoothstep(a));self.target=pour_cup_pose(self.receiver_pose(),self.s,self.tilt)@self.grasp
        else:self.target=blend(self.start,self.phase_goal(),a)
        left_error=self.servos['left'].step(self.left_target)
        self.last_error=self.servos['right'].step(self.target)
        # Gravity compensation reduces position error without extreme PD gains.
        view=self.robot.root_physx_view
        getter=getattr(view,'get_gravity_compensation_forces',None) or getattr(view,'get_generalized_gravity_forces',None)
        if getter is None:raise RuntimeError('PhysX gravity compensation API unavailable.')
        self.robot.set_joint_effort_target(getter())
        closing=self.phase not in ['settle','pregrasp','approach','release','retreat']
        target_finger=max(-.018,self.fit-.0025) if closing else self.s['pour']['open_finger_m']
        self.finger_command+=float(np.clip(target_finger-self.finger_command,-.018*self.dt,.018*self.dt))
        self.robot.set_joint_position_target(torch.full((1,2),self.finger_command,device=self.robot.device),joint_ids=self.finger_ids)
        self.robot.set_joint_velocity_target(torch.zeros((1,2),device=self.robot.device),joint_ids=self.finger_ids)
        if self.ticks%60==0:self.write_log('RUNNING')
        pe,ae=self.last_error;reached=pe<.006 and ae<.07 and left_error[0]<.012
        self.stable=self.stable+self.dt if reached else 0.
        received=self.water and self.measurement.get('received_ml',0)>=self.water.ml*self.s['pour']['target_received_fraction']
        if self.phase=='tilt' and received and self.tilt>45:
            self.stop_tilt=self.tilt;self.advance(to='hold');return
        if a<1 or self.stable<.20:return
        if self.phase=='settle':
            if self.water and self.measurement.get('readback_displacement_m',0)<1e-5:
                self.fail('Particle state did not update. Use CPU tensor pipeline with GPU dynamics and USD readback.');return
            self.source_start=self.source_pose()
        if self.phase=='close':
            measured=self.robot.data.joint_pos[0,self.finger_ids].cpu().numpy()
            if np.max(np.abs(measured-self.fit))>.006:
                if self.phase_elapsed<4:return
                self.fail('Finger opening did not match the cup; grasp not established.');return
            self.grasp=np.linalg.inv(self.source_pose())@self.servos['right'].actual()
        if self.phase=='test_lift' and self.source_pose()[2,3]-self.source_start[2,3]<.015:
            self.fail('Cup did not follow the physical gripper during the 25 mm lift.');return
        if self.phase=='hold' and self.water and not received:
            if self.phase_elapsed<8:return
            self.fail('Insufficient water reached receiver.');return
        if self.phase=='lower':
            if self.source_pose()[2,3]>self.source_start[2,3]+.008:return
            if np.linalg.norm(self.source.data.root_state_w[0,7:10].cpu().numpy())>.03:return
        if self.phase=='retreat':
            actual=self.source_pose();returned=np.linalg.norm(actual[:3,3]-self.source_start[:3,3])<.025 and actual[2,2]>.98
            self.done=True;self.failed=not returned or bool(self.water and not received)
            self.reason='Measured water transfer and cup return completed.' if self.water and not self.failed else 'Dry motion finished; no water transfer was tested.' if not self.water and returned else 'Final task conditions not met.'
            self.write_log('SUCCESS' if not self.failed else 'FAILED',self.reason);print('[POUR]',self.reason,flush=True);return
        self.advance()

    def advance(self,to=None):
        self.phase_idx=next(i for i,(n,_) in enumerate(self.phases) if n==to) if to else self.phase_idx+1
        self.phase=self.phases[self.phase_idx][0];self.phase_time=0.;self.phase_elapsed=0.;self.stable=0.
        self.start=self.servos['right'].actual();self.last_error=(0.,0.)
        print('[POUR]',self.phase,flush=True)

    def close(self):self.log.close()
