"""Build a metric tabletop assets. No image-to-3D model is used.

The table dimensions are user confirmed; all other dimensions/physics are estimates.
Dependencies (separate asset-authoring environment): numpy, trimesh, scipy, usd-core.
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

PACK = Path(__file__).resolve().parents[1]
MATERIALS = {
    "table": ((0.87, 0.86, 0.77), 0.05, 0.48, 1.0),
    "edge": ((0.76, 0.76, 0.69), 0.0, 0.62, 1.0),
    "steel": ((0.56, 0.60, 0.61), 0.88, 0.23, 1.0),
    "steel_dark": ((0.23, 0.28, 0.29), 0.82, 0.30, 1.0),
    "white": ((0.89, 0.90, 0.86), 0.0, 0.48, 1.0),
    "paper": ((0.85, 0.85, 0.80), 0.0, 0.83, 1.0),
    "glass": ((0.66, 0.81, 0.80), 0.0, 0.13, 0.28),
    "glass_rim": ((0.63, 0.76, 0.75), 0.0, 0.16, 0.65),
    "blue": ((0.28, 0.61, 0.76), 0.0, 0.42, 1.0),
    "cream": ((0.77, 0.77, 0.55), 0.0, 0.47, 1.0),
    "black": ((0.018, 0.025, 0.026), 0.0, 0.58, 1.0),
    "red": ((0.62, 0.022, 0.035), 0.0, 0.45, 1.0),
    "brown": ((0.22, 0.11, 0.055), 0.0, 0.88, 1.0),
    "grey": ((0.24, 0.28, 0.28), 0.0, 0.9, 1.0),
    "carpet": ((0.16, 0.19, 0.19), 0.0, 1.0, 1.0),
    "stripe": ((0.25, 0.28, 0.27), 0.0, 1.0, 1.0),
    "stripe_red": ((0.24, 0.12, 0.105), 0.0, 1.0, 1.0),
}


def transform(mesh, pos=(0, 0, 0), yaw=0):
    mesh = mesh.copy()
    mat = trimesh.transformations.rotation_matrix(math.radians(yaw), [0, 0, 1])
    mat[:3, 3] = pos
    mesh.apply_transform(mat)
    return mesh


def box(size, pos=(0, 0, 0), yaw=0):
    return transform(trimesh.creation.box(extents=size), pos, yaw)


def cyl(radius, height, pos=(0, 0, 0), n=64):
    return transform(trimesh.creation.cylinder(radius=radius, height=height, sections=n), pos)


def ellipsoid(size, pos=(0, 0, 0), yaw=0):
    m = trimesh.creation.icosphere(subdivisions=2)
    m.apply_scale(np.array(size) / 2)
    return transform(m, pos, yaw)


def lathe(profile, n=96):
    m = trimesh.creation.revolve(np.array(profile, dtype=float), sections=n)
    m.fix_normals()
    return m


def shell(rb, rt, h, t, n=96):
    return lathe([(0, 0), (rb, 0), (rt, h), (rt-t, h), (rb-t, t), (0, t)], n)


def torus(major, minor, pos, rotation=None):
    m = trimesh.creation.torus(major_radius=major, minor_radius=minor,
                              major_sections=64, minor_sections=10)
    if rotation is not None:
        m.apply_transform(rotation)
    return transform(m, pos)


def segment(a, b, radius, n=12):
    return trimesh.creation.cylinder(radius=radius, segment=np.array([a,b]), sections=n)


def smooth_handle(points,radius):
    vertices=[];faces=[];n=16
    for i,p in enumerate(points):
        tangent=points[min(i+1,len(points)-1)]-points[max(i-1,0)]
        tangent=tangent/np.linalg.norm(tangent)
        normal=np.array([tangent[2],0,-tangent[0]])
        for a in np.linspace(0,2*math.pi,n,endpoint=False):
            vertices.append(p+radius*(normal*math.cos(a)+np.array([0,1,0])*math.sin(a)))
    for i in range(len(points)-1):
        for j in range(n):
            a=i*n+j;b=i*n+(j+1)%n
            faces.extend([[a,b,b+n],[a,b+n,a+n]])
    vertices.extend([points[0],points[-1]])
    for j in range(n):
        faces.extend([[len(vertices)-2,(j+1)%n,j],
                      [len(vertices)-1,(len(points)-1)*n+j,(len(points)-1)*n+(j+1)%n]])
    m=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    m.fix_normals()
    return m


def rounded_slab(w, d, h, radius, z):
    vertices = []
    for zz in [z-h/2, z+h/2]:
        for cx, cy, start in [(w/2-radius,d/2-radius,0),(-w/2+radius,d/2-radius,90),
                              (-w/2+radius,-d/2+radius,180),(w/2-radius,-d/2+radius,270)]:
            for theta in np.linspace(start,start+90,13):
                a=math.radians(theta)
                vertices.append([cx+radius*math.cos(a), cy+radius*math.sin(a), zz])
    return trimesh.convex.convex_hull(np.array(vertices))


@dataclass
class Asset:
    name: str
    label: str
    mass: float = 0
    friction: float = 0.6
    visuals: list = field(default_factory=list)
    colliders: list = field(default_factory=list)

    def add(self, name, mesh, material, collide=False):
        self.visuals.append((name, mesh, material))
        if collide:
            self.col(name,mesh.copy())

    def col(self, name, mesh):
        # At most 32 vertices => at most 60 triangular hull faces, within the
        # 64-vertex / 64-face GPU convex limits. Keep coordinate extrema.
        if len(mesh.vertices)>32:
            v=np.unique(mesh.vertices,axis=0)
            selected=list(dict.fromkeys(np.r_[v.argmin(axis=0),v.argmax(axis=0)].tolist()))
            distance=np.min(np.sum((v[:,None]-v[selected][None,:])**2,axis=2),axis=1)
            while len(selected)<min(32,len(v)):
                index=int(np.argmax(distance));selected.append(index)
                distance=np.minimum(distance,np.sum((v-v[index])**2,axis=1))
            mesh=trimesh.convex.convex_hull(v[selected])
        self.colliders.append((name, mesh))

    def merge(self, other, pos=(0,0,0), yaw=0, prefix=None):
        p=prefix or other.name
        self.visuals.extend((p+'_'+n,transform(m,pos,yaw),mat) for n,m,mat in other.visuals)
        self.colliders.extend((p+'_'+n,transform(m,pos,yaw)) for n,m in other.colliders)


def cup_colliders(asset, rb, rt, h, t, n=32):
    """Closed convex wall sectors, preserving a genuinely empty cup interior."""
    asset.col('bottom', cyl(rb, t, (0,0,t/2), n=48))
    for i in range(n):
        points=[]
        for z,ro in [(t,rb+(rt-rb)*t/h),(h,rt)]:
            for r in [ro-t,ro]:
                for a in [2*math.pi*i/n,2*math.pi*(i+1)/n]:
                    points.append([r*math.cos(a),r*math.sin(a),z])
        asset.col(f'wall_{i:02}',trimesh.convex.convex_hull(np.array(points)))


def make_cup(name, label, rb, rt, h, thickness, material, mass):
    a=Asset(name,label,mass)
    a.add('shell',shell(rb,rt,h,thickness),material)
    a.add('rolled_rim',torus(rt-thickness/2,thickness/2,(0,0,h-thickness/2)),material)
    cup_colliders(a,rb,rt,h,thickness)
    return a


def make_jug(spec):
    rb,rt,h,t=spec['bottom_radius_m'],spec['top_radius_m'],spec['height_m'],spec['wall_m']
    a=make_cup('measuring_jug','透明量杯',rb,rt,h,t,'glass',spec['mass_kg'])
    # Open spout is part of the visual shell; its collision is deliberately approximated.
    body=a.visuals[0][1]
    v=body.vertices.copy()
    az=np.arctan2(v[:,1],v[:,0])
    weight=np.maximum(0,(-np.cos(az)-0.91)/0.09)*np.clip((v[:,2]/h-.78)/.22,0,1)
    v[:,0]-=0.016*weight
    body.vertices=v
    # Hollow D-shaped handle on +X, with real collision opening.
    theta=np.linspace(-math.pi/2,math.pi/2,17)
    pts=np.array([[rt*.92+.042*math.cos(q),0,h*.53+.043*math.sin(q)] for q in theta])
    a.add('handle',smooth_handle(pts,.0048),'glass_rim')
    for i in range(len(pts)-1):
        a.col(f'handle_{i:02}',segment(pts[i],pts[i+1],.0048))
    # Registration marks are geometry, not baked photographic shadows.
    for i in range(1,10):
        z=h*(.16+i*.072)
        r=rb+(rt-rb)*z/h
        a.add(f'mark_{i}',box((.014 if i%2==0 else .008,.00065,.0008),(0,-r-.00025,z)), 'grey')
    return a


def make_basket(name,w,d,h,color):
    a=Asset(name,name)
    t=.0035
    a.add('base',rounded_slab(w,d,t,.013,t/2),color,True)
    # Visual slats suggest the perforated plastic without photograph projection.
    for axis,length,side in [('x',w,d/2),('y',d,w/2)]:
        for sign in [-1,1]:
            for k,z in enumerate([.007,.028,.049,h-.004]):
                dims=(length,.004,.006) if axis=='x' else (.004,length,.006)
                pos=(0,sign*side,z) if axis=='x' else (sign*side,0,z)
                a.add(f'{axis}_{sign}_rail{k}'.replace('-','m'),box(dims,pos),color)
            for k,v in enumerate(np.arange(-length/2+.007,length/2,.015)):
                dims=(.004,.004,h) if axis=='x' else (.004,.004,h)
                pos=(v,sign*side,h/2) if axis=='x' else (sign*side,v,h/2)
                a.add(f'{axis}_{sign}_rib{k}'.replace('-','m'),box(dims,pos),color)
            dims=(length,t,h) if axis=='x' else (t,length,h)
            pos=(0,sign*side,h/2) if axis=='x' else (sign*side,0,h/2)
            a.col(f'{axis}_{sign}_wall'.replace('-','m'),box(dims,pos))
    return a


def make_tray(name,w,d,h):
    a=Asset(name,name)
    a.add('base',rounded_slab(w,d,.004,.015,.002),'steel',True)
    for k,sign in enumerate([-1,1]):
        a.add(f'long{k}',box((w,.003,h),(0,sign*(d/2-.0015),h/2)),'steel',True)
        a.add(f'short{k}',box((.003,d,h),(sign*(w/2-.0015),0,h/2)),'steel',True)
    return a


def bottle(name,r,h,cap='white'):
    a=Asset(name,name)
    a.add('body',lathe([(0,0),(r*.8,0),(r,.015),(r,h*.64),
                       (r*.47,h*.78),(r*.43,h*.93),(0,h*.93)]),'glass')
    a.add('cap',cyl(r*.47,h*.10,(0,0,h*.95)) ,cap)
    a.add('label',lathe([(r*1.001,h*.32),(r*1.001,h*.58)]),'white')
    a.col('body',cyl(r,h,(0,0,h/2)))
    return a


def material(stage,path,name):
    color,metal,rough,opacity=MATERIALS[name]
    mat=UsdShade.Material.Define(stage,path)
    shader=UsdShade.Shader.Define(stage,path+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    for key,typ,val in [('diffuseColor',Sdf.ValueTypeNames.Color3f,Gf.Vec3f(*color)),
                        ('metallic',Sdf.ValueTypeNames.Float,metal),
                        ('roughness',Sdf.ValueTypeNames.Float,rough),
                        ('opacity',Sdf.ValueTypeNames.Float,opacity),
                        ('ior',Sdf.ValueTypeNames.Float,1.49)]:
        shader.CreateInput(key,typ).Set(val)
    shader.CreateOutput('surface',Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    return mat


def usd_mesh(stage,path,mesh,mat=None,collision=False):
    if mat:
        mesh=trimesh.graph.smooth_shade(mesh,angle=math.radians(35),facet_minarea=None)
    u=UsdGeom.Mesh.Define(stage,path)
    u.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(mesh.vertices.astype(np.float32)))
    u.CreateFaceVertexCountsAttr([3]*len(mesh.faces))
    u.CreateFaceVertexIndicesAttr(mesh.faces.ravel().tolist())
    u.CreateSubdivisionSchemeAttr('none')
    u.CreateExtentAttr([Gf.Vec3f(*mesh.bounds[0]),Gf.Vec3f(*mesh.bounds[1])])
    u.CreateDoubleSidedAttr(True)
    if mat:
        UsdShade.MaterialBindingAPI.Apply(u.GetPrim()).Bind(mat)
        # Smooth normals for rounded parts; keep cuboids crisp.
        normals=mesh.vertex_normals[mesh.faces].reshape(-1,3)
        u.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals.astype(np.float32)))
        u.SetNormalsInterpolation('faceVarying')
    if collision:
        u.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(u.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(u.GetPrim()).CreateApproximationAttr('convexHull')
        # Optional PhysX schema names are understood in Isaac Sim; core USD retains them.
        u.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
        u.GetPrim().CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.001)
        u.GetPrim().CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0)
    return u


def new_stage(path,root):
    stage=Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage,1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage,1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/'+root).GetPrim())
    return stage


def export_asset(asset,path):
    root='/'+asset.name
    st=new_stage(path,asset.name)
    st.GetDefaultPrim().SetCustomData({'label':asset.label,'reconstruction':'four_view_parametric_approximation'})
    mats={name:material(st,root+'/Looks/'+name,name) for name in {x[2] for x in asset.visuals}}
    pm=UsdShade.Material.Define(st,root+'/Looks/Contact')
    phy=UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
    phy.CreateStaticFrictionAttr(asset.friction)
    phy.CreateDynamicFrictionAttr(asset.friction*.85)
    phy.CreateRestitutionAttr(0.0)
    for name,m,mat in asset.visuals:
        usd_mesh(st,root+'/Visuals/'+name,m,mats[mat])
    for name,m in asset.colliders:
        u=usd_mesh(st,root+'/Collisions/'+name,m,collision=True)
        UsdShade.MaterialBindingAPI.Apply(u.GetPrim()).Bind(pm,materialPurpose='physics')
    if asset.mass>0:
        prim=st.GetDefaultPrim()
        rb=UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateKinematicEnabledAttr(False)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(asset.mass)
        # COM and inertia are computed by PhysX from the authored compound colliders.
        prim.AddAppliedSchema('PhysxRigidBodyAPI')
        for key,typ,value in [('enableCCD',Sdf.ValueTypeNames.Bool,True),
                              ('solverPositionIterationCount',Sdf.ValueTypeNames.Int,12),
                              ('solverVelocityIterationCount',Sdf.ValueTypeNames.Int,4),
                              ('maxDepenetrationVelocity',Sdf.ValueTypeNames.Float,1.0)]:
            prim.CreateAttribute('physxRigidBody:'+key,typ).Set(value)
    st.GetRootLayer().Save()


