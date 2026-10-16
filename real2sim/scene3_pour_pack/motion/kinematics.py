"""URDF forward kinematics and bounded IK; angles always come from the model."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

def transform(xyz=(0,0,0),rpy=(0,0,0)):
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();T[:3,3]=xyz;return T

def origin(e):
    return transform(*[list(map(float,e.get(k,'0 0 0').split())) for k in ['xyz','rpy']]) if e is not None else np.eye(4)

class G1Kinematics:
    def __init__(self,path=None):
        path=path or Path(__file__).resolve().parents[2]/'robot/g1d_v4/g1_d_upper_dex1_v4.urdf'
        self.path=Path(path);self.xml=ET.parse(path).getroot();self.links={x.get('name'):x for x in self.xml.findall('link')}
        self.joints=[]
        for e in self.xml.findall('joint'):
            limit=e.find('limit');axis=e.find('axis')
            self.joints.append({'name':e.get('name'),'type':e.get('type'),'parent':e.find('parent').get('link'),
                'child':e.find('child').get('link'),'origin':origin(e.find('origin')),
                'axis':np.array(list(map(float,axis.get('xyz').split()))) if axis is not None else np.array([1.,0,0]),
                'lower':float(limit.get('lower','-3.14')) if limit is not None else 0,
                'upper':float(limit.get('upper','3.14')) if limit is not None else 0,
                'effort':float(limit.get('effort','0')) if limit is not None else 0,
                'velocity':float(limit.get('velocity','0')) if limit is not None else 0})
        children={j['child'] for j in self.joints};self.root=next(n for n in self.links if n not in children)
        self.by_name={j['name']:j for j in self.joints}

    def fk(self,q=None,base=None):
        q=q or {};out={self.root:np.eye(4) if base is None else np.array(base)};remaining=self.joints.copy()
        while remaining:
            before=len(remaining)
            for j in remaining[:]:
                if j['parent'] not in out:continue
                move=np.eye(4);v=q.get(j['name'],0)
                if j['type'] in ['revolute','continuous']:move[:3,:3]=Rotation.from_rotvec(j['axis']*v).as_matrix()
                elif j['type']=='prismatic':move[:3,3]=j['axis']*v
                out[j['child']]=out[j['parent']]@j['origin']@move;remaining.remove(j)
            if len(remaining)==before:raise ValueError('Disconnected URDF')
        return out

    def names(self,side):
        return [side+'_'+n+'_joint' for n in ['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']]

    def tcp(self,side,q,base=None,offset=(.083,0,0)):
        return self.fk(q,base)[side+'_dex1_base_link']@transform(offset)

    def solve(self,side,target,seed=None,base=None,offset=(.083,0,0)):
        names=self.names(side);seed=np.zeros(7) if seed is None else np.array(seed)
        lo=np.array([self.by_name[n]['lower'] for n in names])+.005
        hi=np.array([self.by_name[n]['upper'] for n in names])-.005
        seed=np.clip(seed,lo+.0001,hi-.0001)
        def residual(q):
            T=self.tcp(side,dict(zip(names,q)),base,offset)
            return np.r_[10*(T[:3,3]-target[:3,3]),Rotation.from_matrix(target[:3,:3].T@T[:3,:3]).as_rotvec(),.015*(q-seed)]
        result=least_squares(residual,seed,bounds=(lo,hi),max_nfev=140,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        T=self.tcp(side,dict(zip(names,result.x)),base,offset)
        error=np.linalg.norm(T[:3,3]-target[:3,3]);angle=np.linalg.norm(Rotation.from_matrix(target[:3,:3].T@T[:3,:3]).as_rotvec())
        return result.x,{'position_error_m':float(error),'rotation_error_deg':float(np.degrees(angle))}
