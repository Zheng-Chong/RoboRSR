"""Convert the downloaded official G1 URDF to a self-contained articulated USD."""
from pathlib import Path
import sys,json,math
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'motion'))
from kinematics import G1Kinematics,origin
from geometry_utils import new_stage,usd_mesh,MATERIALS,material,Asset,cyl,box,ellipsoid
P=Path(__file__).resolve().parents[1]

def quat(R):
    q=Rotation.from_matrix(R).as_quat();return Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3])))

def set_pose(x,T):
    x.AddTranslateOp().Set(Gf.Vec3d(*T[:3,3]));x.AddOrientOp().Set(quat(T[:3,:3]))

def geometry(e,folder):
    if e.find('mesh') is not None:
        m=trimesh.load_mesh(folder/e.find('mesh').get('filename'),process=True)
        m.apply_scale(list(map(float,e.find('mesh').get('scale','1 1 1').split())))
    elif e.find('sphere') is not None:m=trimesh.creation.icosphere(subdivisions=1,radius=float(e.find('sphere').get('radius')))
    elif e.find('cylinder') is not None:
        c=e.find('cylinder');m=cyl(float(c.get('radius')),float(c.get('length')),n=24)
    else:m=box(list(map(float,e.find('box').get('size').split())))
    return m

def main():
    kin=G1Kinematics();frames=kin.fk();folder=kin.path.parent
    st=new_stage(P/'robot/g1_dex1.usdc','G1');root='/G1'
    UsdPhysics.ArticulationRootAPI.Apply(st.GetDefaultPrim());st.GetDefaultPrim().AddAppliedSchema('PhysxArticulationAPI')
    for key,typ,val in [('enabledSelfCollisions',Sdf.ValueTypeNames.Bool,True),('solverPositionIterationCount',Sdf.ValueTypeNames.Int,32),('solverVelocityIterationCount',Sdf.ValueTypeNames.Int,8)]:st.GetDefaultPrim().CreateAttribute('physxArticulation:'+key,typ).Set(val)
    st.GetDefaultPrim().SetCustomData({'source':'unitreerobotics/unitree_ros','model':kin.path.name,'note':'Official URDF geometry and kinematics; convex collision authoring and USD conversion by this pack.'})
    MATERIALS.update({'g1_white':((.65,.67,.69),.15,.36,1),'g1_dark':((.028,.035,.045),.25,.44,1),'g1_finger':((.15,.16,.17),.25,.56,1)})
    mats={n:material(st,root+'/Looks/'+n,n) for n in ['g1_white','g1_dark','g1_finger']}
    physics=UsdShade.Material.Define(st,root+'/Looks/Contact');api=UsdPhysics.MaterialAPI.Apply(physics.GetPrim());api.CreateStaticFrictionAttr(.9);api.CreateDynamicFrictionAttr(.75);api.CreateRestitutionAttr(0)
    contact_bounds={}
    for name,e in kin.links.items():
      path=root+'/'+name;x=UsdGeom.Xform.Define(st,path);set_pose(x,frames[name]);UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
      x.GetPrim().AddAppliedSchema('PhysxRigidBodyAPI');x.GetPrim().CreateAttribute('physxRigidBody:enableCCD',Sdf.ValueTypeNames.Bool).Set(True)
      inertial=e.find('inertial');mass=UsdPhysics.MassAPI.Apply(x.GetPrim())
      if inertial is not None:
        mass.CreateMassAttr(float(inertial.find('mass').get('value')));T=origin(inertial.find('origin'));mass.CreateCenterOfMassAttr(Gf.Vec3f(*T[:3,3]))
        ia=inertial.find('inertia');I=np.array([[float(ia.get('i'+i+j if i<=j else 'i'+j+i)) for j in 'xyz'] for i in 'xyz'])
        eig,V=np.linalg.eigh(T[:3,:3]@I@T[:3,:3].T)
        if np.linalg.det(V)<0:V[:,0]*=-1
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.maximum(eig,1e-8)));mass.CreatePrincipalAxesAttr(quat(V))
      else:mass.CreateMassAttr(.001);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-7))
      for i,v in enumerate(e.findall('visual')):
        m=geometry(v.find('geometry'),folder);m.apply_transform(origin(v.find('origin')))
        mat=v.find('material');mn='g1_dark' if mat is not None and mat.get('name')=='dark' else 'g1_finger' if 'finger' in name else 'g1_white'
        usd_mesh(st,path+'/Visuals/Mesh'+str(i),m,mats[mn])
      if name=='head_link':
        # Cosmetic opaque backing fills the open visor area in the supplied CAD.
        # No articulation, mass or collision properties are changed by this panel.
        visor=usd_mesh(st,path+'/Visuals/VisorBacking',ellipsoid((.025,.110,.106),(.041,0,.432)),mats['g1_dark'])
        visor.GetPrim().SetCustomData({'role':'cosmetic visor backing; not part of official URDF meshes'})
      for i,c in enumerate(e.findall('collision')):
        m=geometry(c.find('geometry'),folder);m.apply_transform(origin(c.find('origin')))
        if 'dex1_finger' in name:contact_bounds[name]=m.bounds.tolist()
        # The official finger collision mesh is kept as multiple connected convex pieces.
        pieces=m.split(only_watertight=False) if 'dex1_finger' in name else [m]
        if len(pieces)>30:pieces=[m]
        for k,piece in enumerate(pieces):
          if len(piece.vertices)<4:continue
          a=Asset('c','c');a.col('c',piece.convex_hull)
          u=usd_mesh(st,path+f'/Collisions/C{i}_{k}',a.colliders[0][1],collision=True)
          UsdShade.MaterialBindingAPI.Apply(u.GetPrim()).Bind(physics,materialPurpose='physics')
      print('G1 link',name,flush=True)
    anchor=UsdPhysics.FixedJoint.Define(st,root+'/Joints/root_fixed');anchor.CreateBody1Rel().SetTargets([Sdf.Path(root+'/'+kin.root)])
    anchor.CreateLocalPos0Attr(Gf.Vec3f(0));anchor.CreateLocalPos1Attr(Gf.Vec3f(0))
    for j in kin.joints:
      path=root+'/Joints/'+j['name'];kind=j['type']
      joint=UsdPhysics.FixedJoint.Define(st,path) if kind=='fixed' else UsdPhysics.PrismaticJoint.Define(st,path) if kind=='prismatic' else UsdPhysics.RevoluteJoint.Define(st,path)
      joint.CreateBody0Rel().SetTargets([Sdf.Path(root+'/'+j['parent'])]);joint.CreateBody1Rel().SetTargets([Sdf.Path(root+'/'+j['child'])])
      A=np.eye(3) if kind=='fixed' else trimesh.geometry.align_vectors([1,0,0],j['axis'])[:3,:3]
      joint.CreateLocalPos0Attr(Gf.Vec3f(*j['origin'][:3,3]));joint.CreateLocalRot0Attr(quat(j['origin'][:3,:3]@A));joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot1Attr(quat(A))
      joint.CreateCollisionEnabledAttr(False)
      if kind!='fixed':
        joint.CreateAxisAttr('X');factor=1 if kind=='prismatic' else 180/math.pi
        joint.CreateLowerLimitAttr(j['lower']*factor);joint.CreateUpperLimitAttr(j['upper']*factor)
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear' if kind=='prismatic' else 'angular')
        drive.CreateTypeAttr('force');drive.CreateTargetPositionAttr(0);drive.CreateTargetVelocityAttr(0)
        # Angular USD drives use degrees. Isaac Lab replaces actuator gains at runtime.
        kp,kd=(2500,35) if kind=='prismatic' else (300,8) if any(s in j['name'] for s in ['shoulder','elbow','wrist']) else (1500,70)
        drive.CreateStiffnessAttr(kp/factor);drive.CreateDampingAttr(kd/factor);drive.CreateMaxForceAttr(j['effort'])
    st.GetRootLayer().Save()
    (P/'robot/contact_geometry.json').write_text(json.dumps(contact_bounds,indent=2))
    # Derive a grounded pelvis height from the official foot collision geometry.
    foot_z=[]
    for n in ['left_ankle_roll_link','right_ankle_roll_link']:
      for c in kin.links[n].findall('collision'):
        m=geometry(c.find('geometry'),folder);m.apply_transform(frames[n]@origin(c.find('origin')));foot_z.append(m.bounds[0,2])
    spec=json.loads((P/'scene_spec.json').read_text(encoding='utf-8'));spec['robot']['root_position'][2]=float(-min(foot_z)+.001)
    spec['pour']['max_tilt_deg']=105
    (P/'scene_spec.json').write_text(json.dumps(spec,ensure_ascii=False,indent=2),encoding='utf-8')
    main=Usd.Stage.Open(str(P/'assets/scene3.usda'));main.GetRootLayer().Export(str(P/'assets/scene3_g1.usda'));main=Usd.Stage.Open(str(P/'assets/scene3_g1.usda'))
    robot=UsdGeom.Xform.Define(main,'/World/Robot');robot.GetPrim().GetReferences().AddReference('../robot/g1_dex1.usdc');robot.AddTranslateOp().Set(Gf.Vec3d(*spec['robot']['root_position']))
    UsdPhysics.FixedJoint(main.GetPrimAtPath('/World/Robot/Joints/root_fixed')).GetLocalPos0Attr().Set(Gf.Vec3f(*spec['robot']['root_position']))
    main.GetRootLayer().Save()
    print('Robot USD created. Grounded root height',spec['robot']['root_position'][2])

if __name__=='__main__':main()
