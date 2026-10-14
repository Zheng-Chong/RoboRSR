"""Export actual USD meshes and URDF joints for an offline asset viewer."""
from pathlib import Path
import sys,json,base64
import numpy as np
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Gf
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P/'motion'))
from kinematics import G1Kinematics,transform

def b64(a,dtype):return base64.b64encode(np.asarray(a,dtype=dtype).tobytes()).decode()

def main():
    spec=json.loads((P/'scene_spec.json').read_text(encoding='utf-8'));ready=json.loads((P/'motion/ready_pose.json').read_text())['joint_positions'];kin=G1Kinematics()
    stage=Usd.Stage.Open(str(P/'assets/scene3_g1.usda'));frames=kin.fk(ready)
    for name,T in frames.items():
      p=stage.GetPrimAtPath('/World/Robot/'+name);x=UsdGeom.Xformable(p);ops=x.GetOrderedXformOps()
      for op in ops:
        if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(*T[:3,3]))
        if op.GetOpType()==UsdGeom.XformOp.TypeOrient:
            q=Rotation.from_matrix(T[:3,:3]).as_quat();op.Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
    for name,value in ready.items():
      prim=stage.GetPrimAtPath('/World/Robot/Joints/'+name);kind='linear' if kin.by_name[name]['type']=='prismatic' else 'angular'
      UsdPhysics.DriveAPI(prim,kind).GetTargetPositionAttr().Set(float(value if kind=='linear' else np.degrees(value)))
    stage.GetRootLayer().Save()
    cache=UsdGeom.XformCache();meshes=[];owners={}
    for p in stage.Traverse():
      if not p.IsA(UsdGeom.Mesh):continue
      parts=str(p.GetPath()).split('/');owner='/'.join(parts[:4]) if parts[2]=='Robot' else '/'.join(parts[:3]);coll=p.HasAPI(UsdPhysics.CollisionAPI)
      if owner not in owners:
        T=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(owner)),float).T
        owners[owner]={'matrix':T.ravel().tolist(),'robot_link':parts[3] if parts[2]=='Robot' else None,'name':parts[3] if parts[2]=='Robot' else parts[2]}
      mesh=UsdGeom.Mesh(p);v=np.asarray(mesh.GetPointsAttr().Get(),np.float64);local=np.asarray(cache.GetLocalToWorldTransform(p)*cache.GetLocalToWorldTransform(stage.GetPrimAtPath(owner)).GetInverse())
      v=(np.c_[v,np.ones(len(v))]@local)[:,:3];idx=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),np.uint32)
      color=[.2,.7,.6];metal=0;rough=.5
      if not coll:
        mat,_=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial();shader=UsdShade.Shader(stage.GetPrimAtPath(str(mat.GetPath())+'/Surface'))
        color=list(shader.GetInput('diffuseColor').Get());metal=shader.GetInput('metallic').Get();rough=shader.GetInput('roughness').Get()
      normals=np.asarray(mesh.GetNormalsAttr().Get(),float) if not coll else np.empty((0,3))
      normal_data=None
      if len(normals)==len(idx):
          v=v[idx];idx=None
          normals=normals@np.linalg.inv(local[:3,:3]).T
          normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
          normal_data=b64(normals,'<f4')
      meshes.append({'owner':owner,'name':p.GetName(),'collision':coll,'p':b64(v,'<f4'),'i':b64(idx,'<u4') if idx is not None else None,'n':normal_data,'color':color,'metal':metal,'rough':rough})
    joints=[{**j,'origin':j['origin'].ravel().tolist(),'axis':j['axis'].tolist()} for j in kin.joints]
    motion=json.loads((P/'motion/preview_trajectory.json').read_text())
    data={'spec':spec,'owners':owners,'meshes':meshes,'joints':joints,'root_link':kin.root,'ready':ready,'motion':motion}
    (P/'preview/scene_data.js').write_text('window.POUR_SCENE='+json.dumps(data,ensure_ascii=False,separators=(',',':'))+';',encoding='utf-8')
    print('Asset viewer exported:',len(meshes),'meshes. Animation is explicitly kinematic intent only.')

if __name__=='__main__':main()
