"""Photo-informed, metric Scene 3 assets. No simulated outcomes are claimed."""
from pathlib import Path
import json,math
import numpy as np
import trimesh
from pxr import Gf,Usd,UsdGeom,UsdPhysics,UsdLux,Sdf
from geometry_utils import *
PACK=Path(__file__).resolve().parents[1]
MATERIALS.update({
 'enamel':((.84,.85,.81),0,.32,1), 'metal_inner':((.56,.58,.60),.94,.27,1),
 'metal_edge':((.68,.71,.73),.96,.16,1), 'basket':((.81,.76,.55),0,.45,1),
 'grip':((.43,.44,.43),.05,.72,1), 'fur':((.20,.070,.022),0,.98,1),
 'fur_light':((.28,.11,.045),0,.98,1), 'turtle':((.29,.56,.09),0,.88,1),
 'turtle_dark':((.12,.29,.035),0,.94,1), 'paper_light':((.89,.90,.91),0,.95,1),
 'paper_shadow':((.61,.65,.69),0,.92,1), 'table_white':((.86,.88,.87),0,.50,1),
})

def cup(name,c):
    r0,r1,h,t,b=c['radius_bottom'],c['radius_top'],c['height'],c['wall'],c['bottom']
    a=Asset(name,c['label'],c['mass'],.8)
    outer=[(0,.001),(r0-.002,.001),(r0-.0006,.0015),(r0,.003),
           (r0+.0001,.008),(r1-.0002,h-.002),(r1,h-.001),(r1-.0003,h)]
    a.add('enamel_body',lathe(outer,192),'enamel')
    a.add('steel_inside',lathe([(r1-t,h),(r0-t+.0005,b+.003),(r0-t-.002,b),(0,b)],192),'metal_inner')
    a.add('rolled_steel_rim',lathe([(r1-.0003,h),(r1-.0007,h+.0004),(r1-t+.0006,h+.0004),(r1-t,h)],192),'metal_edge')
    a.add('foot_ring',torus(r0-.001,.00065,(0,0,.001)),'metal_inner')
    # A narrow brushed ring at the mouth, present as geometry rather than a decal.
    a.add('lip_band',lathe([(r1-.0003,h-.0017),(r1+.00005,h-.0005)],192),'metal_edge')
    cup_colliders(a,r0,r1,h,t,n=48)
    a.colliders[0]=('bottom',cyl(r0,b,(0,0,b/2),n=32))
    return a

def perforated_panel(w,h,spacing,radius,thickness,mapping):
    """Cell annuli: circular through holes with bevel-free thin inner walls."""
    nx,ny=max(1,round(w/spacing)),max(1,round(h/spacing))
    sx,sy=w/nx,h/ny;verts=[];faces=[];N=16
    for i in range(nx):
      for j in range(ny):
        cx=-w/2+(i+.5)*sx;cy=-h/2+(j+.5)*sy
        start=len(verts)
        for layer in [-thickness/2,thickness/2]:
          for outer in [True,False]:
            for k in range(N):
              ang=2*math.pi*k/N;co,si=math.cos(ang),math.sin(ang)
              r=min(sx/2/max(abs(co),1e-8),sy/2/max(abs(si),1e-8)) if outer else radius
              verts.append(mapping(cx+r*co,cy+r*si,layer))
        for k in range(N):
          n=(k+1)%N
          faces.extend([[start+k,start+n,start+N+n],[start+k,start+N+n,start+N+k],
                        [start+2*N+k,start+3*N+n,start+2*N+n],[start+2*N+k,start+3*N+k,start+3*N+n],
                        [start+N+k,start+N+n,start+3*N+n],[start+N+k,start+3*N+n,start+3*N+k]])
    m=trimesh.Trimesh(vertices=verts,faces=faces,process=True);m.fix_normals();return m

def basket(name,c):
    x,y,h=c['size'];t=.003;a=Asset(name,c['label'],c['mass'],.6)
    # X is the long side of each basket, extending along table depth.
    a.add('perforated_bottom',perforated_panel(x-.028,y-.028,.015,.0028,t,lambda u,v,d:[u,v,.006+d]),'basket')
    a.col('bottom',box((x-.012,y-.012,.006),(0,0,.003)))
    for axis,span,half in [('x',x,y/2),('y',y,x/2)]:
      for sign in [-1,1]:
        def mapping(u,v,d):
            z=v+(h+.014)/2
            side=sign*(half-.015+.015*z/h+d)
            return [u,side,z] if axis=='x' else [side,u,z]
        a.add(f'wall_{axis}_{sign}'.replace('-','m'),perforated_panel(span-.016,h-.014,.014,.0025,t,mapping),'basket')
        # Continuous tapered convex wall for collision; perforations are visual detail.
        pts=[]
        for z in [0,h]:
          for u in [-span/2,span/2]:
            for d in [-t/2,t/2]:
              s=sign*(half-.015+.015*z/h+d)
              pts.append([u,s,z] if axis=='x' else [s,u,z])
        a.col(f'wallcol_{axis}_{sign}'.replace('-','m'),trimesh.convex.convex_hull(pts))
        p=[0,sign*half,h] if axis=='x' else [sign*half,0,h]
        size=[span,.008,.009] if axis=='x' else [.008,span,.009]
        a.add(f'rim_{axis}_{sign}'.replace('-','m'),box(size,p),'basket')
    for sign in [-1,1]:
      a.add(f'handle_{sign}'.replace('-','m'),box((.085,.009,.013),(0,sign*y/2,h+.001)),'grip')
      for k in range(12):a.add(f'handle_rib_{sign}_{k}'.replace('-','m'),box((.0012,.0095,.0008),(-.038+k*.0068,sign*y/2,h+.008)),'white')
    return a

def fur_mesh(mesh,rng,density=1800):
    # Short geometric fibres follow the surface; they do not participate in physics.
    tri=mesh.triangles;area=mesh.area_faces
    ids=rng.choice(len(tri),size=density,p=area/area.sum());uv=rng.random((density,2));uv[uv.sum(axis=1)>1]=1-uv[uv.sum(axis=1)>1]
    p=tri[ids,0]+uv[:,0,None]*(tri[ids,1]-tri[ids,0])+uv[:,1,None]*(tri[ids,2]-tri[ids,0])
    n=mesh.face_normals[ids];tangent=np.cross(n,np.tile([0,0,1],(density,1)));tangent/=np.maximum(np.linalg.norm(tangent,axis=1,keepdims=True),1e-9)
    ends=p+n*rng.uniform(.0005,.0016,(density,1))+tangent*rng.normal(0,.0004,(density,1))
    verts=np.stack([p-tangent*.00014,p+tangent*.00014,ends],axis=1).reshape(-1,3)
    return trimesh.Trimesh(vertices=verts,faces=np.arange(len(verts)).reshape(-1,3),process=False)

def toy(c):
    a=Asset('toy',c['label'],c['mass'],.9);rng=np.random.default_rng(33)
    parts=[('body',(.09,.12,.075),(0,-.013,.042)),('head',(.076,.077,.063),(0,.050,.044)),
           ('snout',(.063,.038,.033),(0,.086,.030)),('ear_l',(.017,.022,.026),(-.028,.042,.072)),('ear_r',(.017,.022,.026),(.028,.042,.072))]
    for n,size,p in parts:
      m=trimesh.creation.icosphere(subdivisions=3);m.apply_scale(np.array(size)/2);m.apply_translation(p)
      a.add(n,m,'fur_light' if n=='snout' else 'fur',True);a.add(n+'_fibres',fur_mesh(m,rng,3000 if n=='body' else 900),'fur')
    for i,(x,y) in enumerate([(-.031,-.045),(.031,-.045),(-.031,.026),(.031,.026)]):a.add(f'foot{i}',ellipsoid((.027,.035,.021),(x,y,.0105)),'fur',True)
    for sign in [-1,1]:
      a.add(f'eye{sign}'.replace('-','m'),ellipsoid((.003,.003,.003),(sign*.030,.075,.050)),'black')
      a.add(f'nostril{sign}'.replace('-','m'),ellipsoid((.002,.002,.0015),(sign*.014,.103,.035)),'black')
    # Green turtle accessory visible on the back in all four photos.
    a.add('turtle_shell',ellipsoid((.043,.050,.015),(0,-.014,.086)),'turtle')
    a.add('turtle_head',ellipsoid((.018,.022,.012),(0,.019,.085)),'turtle')
    for sign in [-1,1]:
      for y in [-.032,.002]:a.add(f'turtle_foot_{sign}_{y}'.replace('-','m').replace('.','p'),ellipsoid((.020,.012,.007),(sign*.023,y,.081),sign*20),'turtle')
      a.add(f'turtle_eye_{sign}'.replace('-','m'),ellipsoid((.0015,.0015,.0015),(sign*.004,.027,.091)),'black')
      a.add(f'strap_{sign}'.replace('-','m'),segment((sign*.012,-.030,.085),(sign*.042,.017,.055),.0024,12),'turtle')
    for x in [-.008,.008]:a.add('shell_line'+str(x).replace('-','m').replace('.','p'),segment((x,-.031,.093),(x,.001,.093),.00065,8),'turtle_dark')
    return a

def paper(c):
    a=Asset('paper',c['label'],c['mass'],.6);rng=np.random.default_rng(24)
    m=trimesh.creation.icosphere(subdivisions=2);v=m.vertices.copy();v*=rng.uniform(.55,1.20,(len(v),1));v*=np.array([.023,.027,.025]);v[:,2]-=v[:,2].min();m.vertices=v
    a.add('folded_core',m,'paper_light');a.col('core',m.convex_hull)
    for i in range(24):
      center=np.array([rng.uniform(-.017,.017),rng.uniform(-.019,.019),rng.uniform(.010,.042)])
      points=center+rng.normal(0,1,(5,3))*[.010,.014,.008]
      points[:,2]=np.maximum(points[:,2],.001)
      a.add(f'crease{i}',trimesh.Trimesh(vertices=points,faces=[[0,1,2],[0,2,3],[0,3,4]],process=False),'paper_shadow' if i%5==0 else 'paper_light')
    return a

def table(spec):
    a=Asset('Table','桌子与地面');h=spec['table']['height_m']
    a.add('top',rounded_slab(.6,1.2,.025,.035,h-.0125),'table_white',True)
    for y in [-.47,.47]:
      s='left' if y>0 else 'right'
      a.add(s+'_column',cyl(.027,h-.060,(0,y,(h-.060)/2+.027)),'white',True)
      a.add(s+'_foot',rounded_slab(.53,.065,.034,.026,.019).apply_translation([0,y,0]),'white',True)
      a.add(s+'_crossbar',box((.48,.06,.030),(0,y,h-.042)),'steel',True)
    a.add('beam',box((.027,.95,.033),(0,0,h-.048)),'steel_dark',True)
    a.add('floor',box((3,3,.03),(0,0,-.015)),'carpet',True)
    return a

def place(stage,path,assetfile,position,yaw=0,kinematic=False):
    p=UsdGeom.Xform.Define(stage,path);p.GetPrim().GetReferences().AddReference(assetfile)
    p.AddTranslateOp().Set(Gf.Vec3d(*position));p.AddOrientOp().Set(Gf.Quatf(math.cos(math.radians(yaw)/2),Gf.Vec3f(0,0,math.sin(math.radians(yaw)/2))))
    if kinematic:UsdPhysics.RigidBodyAPI(p.GetPrim()).CreateKinematicEnabledAttr(True)
    return p

def main():
    spec=json.loads((PACK/'scene_spec.json').read_text(encoding='utf-8'));out=PACK/'assets';out.mkdir(exist_ok=True)
    export_asset(table(spec),out/'table.usdc')
    for n,c in spec['objects'].items():
      a=cup(n,c) if n.endswith('cup') else basket(n,c) if n.startswith('basket') else toy(c) if n=='toy' else paper(c)
      export_asset(a,out/'objects'/f'{n}.usdc')
      print(n,len(a.visuals),'visuals',len(a.colliders),'colliders',flush=True)
    st=new_stage(out/'scene3.usda','World')
    place(st,'/World/Table','./table.usdc',[0,0,0])
    for n,c in spec['objects'].items():place(st,'/World/'+n,'./objects/'+n+'.usdc',c['position'],c['yaw'],not n.endswith('cup'))
    scene=UsdPhysics.Scene.Define(st,'/World/PhysicsScene');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    scene.GetPrim().AddAppliedSchema('PhysxSceneAPI')
    for n,t,v in [('enableCCD',Sdf.ValueTypeNames.Bool,True),('enableGPUDynamics',Sdf.ValueTypeNames.Bool,True),('broadphaseType',Sdf.ValueTypeNames.Token,'GPU'),('timeStepsPerSecond',Sdf.ValueTypeNames.UInt,240),('solverType',Sdf.ValueTypeNames.Token,'TGS')]:scene.GetPrim().CreateAttribute('physxScene:'+n,t).Set(v)
    dome=UsdLux.DomeLight.Define(st,'/World/Lighting/Fill');dome.CreateIntensityAttr(650);dome.CreateColorAttr(Gf.Vec3f(.89,.94,1))
    for n,p,size,power in [('Key',(-.6,-.6,2.5),(1.3,1.1),1800),('Softbox',(.8,.6,2.0),(.7,1.2),900)]:
      light=UsdLux.RectLight.Define(st,'/World/Lighting/'+n);light.CreateWidthAttr(size[0]);light.CreateHeightAttr(size[1]);light.CreateIntensityAttr(power);light.AddTranslateOp().Set(p)
    cam=UsdGeom.Camera.Define(st,'/World/Cameras/Overview');eye=Gf.Vec3d(-1.9,-1.7,1.65);target=Gf.Vec3d(-.1,0,.70)
    cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse());cam.CreateFocalLengthAttr(35);cam.CreateClippingRangeAttr(Gf.Vec2f(.02,100))
    st.GetRootLayer().Save()
    print('Empty photo scene:',out/'scene3.usda')

if __name__=='__main__':main()
