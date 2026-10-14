"""Generate a geometric preview and initial IK pose, not a physics validation."""
from pathlib import Path
import sys,json
import numpy as np
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P/'motion'))
from kinematics import G1Kinematics,transform
from task_geometry import initial_targets,plan_targets,blend,pour_cup_pose,grasp_transform,pose,smoothstep

def main():
    s=json.loads((P/'scene_spec.json').read_text(encoding='utf-8'));s['pour']['tcp_in_dex1_base_m']=[.112,0,0];s['pour']['lip_clearance_m']=.15
    s['pour']['neutral_finger_gap_m']=.0458768345
    (P/'scene_spec.json').write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf-8')
    k=G1Kinematics();base=transform(s['robot']['root_position']);ready=initial_targets(s);q={};infos={}
    for side in ['left','right']:
      value,info=k.solve(side,ready[side],base=base,offset=s['pour']['tcp_in_dex1_base_m']);q.update(dict(zip(k.names(side),value)));infos[side]=info
    for side in ['left','right']:
      for finger in [1,2]:q[f'{side}_dex1_finger_joint_{finger}']=s['pour']['open_finger_m']
    (P/'motion/ready_pose.json').write_text(json.dumps({'joint_positions':q,'method':'Bounded numerical IK from official URDF','note':'Initialization seed, not a tested physics result','ik_residuals':infos},indent=2))
    source=pose(s['objects']['source_cup']['position']);receiver=pose(s['objects']['receiver_cup']['position']);phases=plan_targets(s,source,receiver)
    frames=[];time=0;previous=ready['right'];seed=np.array([q[n] for n in k.names('right')]);grasp=grasp_transform(s)
    for phase,target,duration in phases:
      for step in range(1,13):
        a=step/12
        if phase=='tilt':T=pour_cup_pose(receiver,s,s['pour']['max_tilt_deg']*smoothstep(a))@grasp
        elif phase=='untilt':T=pour_cup_pose(receiver,s,s['pour']['max_tilt_deg']*(1-smoothstep(a)))@grasp
        else:T=blend(previous,target,a)
        seed,info=k.solve('right',T,seed=seed,base=base,offset=s['pour']['tcp_in_dex1_base_m']);q.update(dict(zip(k.names('right'),seed)))
        closed=phase not in ['pregrasp','approach','release','retreat']
        radius=s['objects']['source_cup']['radius_bottom']+(s['objects']['source_cup']['radius_top']-s['objects']['source_cup']['radius_bottom'])*s['pour']['grasp_height_m']/s['objects']['source_cup']['height']
        gapfit=(radius*2-s['pour']['neutral_finger_gap_m'])/2
        for f in [1,2]:q[f'right_dex1_finger_joint_{f}']=gapfit if closed else s['pour']['open_finger_m']
        held=phase not in ['pregrasp','approach','close','release','retreat']
        cup=T@np.linalg.inv(grasp) if held else source
        frames.append({'t':round(time+duration*a,4),'phase':phase,'q':q.copy(),'source_cup_pose':cup.tolist(),'tcp_goal':T.tolist()})
      previous=target;time+=duration
    result={'kind':'kinematic_intent_preview','not_physics_simulation':True,'duration_s':time,'frames':frames}
    (P/'motion/preview_trajectory.json').write_text(json.dumps(result,separators=(',',':')))
    print('Generated IK initialization and geometric motion preview. No physical validation performed.')
    print('Initial numerical IK residuals:',infos)

if __name__=='__main__':main()
