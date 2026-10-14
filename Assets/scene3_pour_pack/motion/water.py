"""PhysX PBD fluid, created in the empty source cup at task startup.

Use GPU dynamics with the CPU tensor pipeline so particle state can be read from
USD on Isaac Sim 4.5/5.x. Rendering never fabricates transferred volume.
"""
import numpy as np
from pxr import Gf,Sdf,UsdGeom,UsdPhysics,UsdShade,Vt

def sample_water(spec,ml):
    c=spec['objects']['source_cup'];d=spec['pour']['particle_spacing_m'];points=[]
    count=int(round(ml*1e-6/d**3))
    for z in np.arange(c['bottom']+d,c['height']-2*d,d):
      r=c['radius_bottom']+(c['radius_top']-c['radius_bottom'])*z/c['height']-c['wall']-d
      layer=[]
      for x in np.arange(-c['radius_top'],c['radius_top']+d,d):
        for y in np.arange(-c['radius_top'],c['radius_top']+d,d):
          if x*x+y*y<r*r:layer.append([x,y,z])
      # Fill partial top layer from the centre outwards.
      layer.sort(key=lambda v:v[0]*v[0]+v[1]*v[1]);points.extend(layer)
      if len(points)>=count:break
    if count<=0:return np.empty((0,3))
    if len(points)<count:raise ValueError('Water volume exceeds conservative cup capacity.')
    return np.array(points[:count])+np.array(c['position'])

class Water:
    def __init__(self,stage,spec,ml,physics_scene_path='/physicsScene'):
        from pxr import PhysxSchema
        self.spec=spec;self.ml=ml;self.stage=stage;self.initial=sample_water(spec,ml);self.n=len(self.initial)
        self.path='/World/Water/Particles';d=spec['pour']['particle_spacing_m'];rest=d/2;contact=rest/(.99*.6)
        system=PhysxSchema.PhysxParticleSystem.Define(stage,'/World/Water/System')
        system.CreateSimulationOwnerRel().SetTargets([Sdf.Path(physics_scene_path)])
        system.CreateParticleContactOffsetAttr(contact);system.CreateFluidRestOffsetAttr(rest)
        system.CreateSolidRestOffsetAttr(contact*.99);system.CreateRestOffsetAttr(rest)
        system.CreateContactOffsetAttr(contact);system.CreateSolverPositionIterationCountAttr(16)
        system.CreateMaxNeighborhoodAttr(96);system.CreateMaxVelocityAttr(4.0)
        system.CreateEnableCCDAttr(True);system.CreateGlobalSelfCollisionEnabledAttr(True)
        system.CreateNonParticleCollisionEnabledAttr(True)
        mat=UsdShade.Material.Define(stage,'/World/Water/Material')
        pbd=PhysxSchema.PhysxPBDMaterialAPI.Apply(mat.GetPrim());pbd.CreateDensityAttr(1000.)
        pbd.CreateFrictionAttr(.05);pbd.CreateViscosityAttr(.01);pbd.CreateCohesionAttr(.001)
        pbd.CreateSurfaceTensionAttr(.01);pbd.CreateDampingAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(system.GetPrim()).Bind(mat,materialPurpose='physics')
        points=UsdGeom.PointInstancer.Define(stage,self.path)
        points.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(self.initial.astype(np.float32)))
        points.CreateVelocitiesAttr(Vt.Vec3fArray.FromNumpy(np.zeros_like(self.initial,dtype=np.float32)))
        points.CreateProtoIndicesAttr([0]*self.n)
        sphere=UsdGeom.Sphere.Define(stage,self.path+'/Sphere');sphere.CreateRadiusAttr(rest)
        sphere.CreateDisplayColorAttr([Gf.Vec3f(.18,.55,.85)])
        points.CreatePrototypesRel().SetTargets([sphere.GetPath()])
        pa=PhysxSchema.PhysxParticleAPI.Apply(points.GetPrim())
        pa.CreateParticleSystemRel().SetTargets([system.GetPath()]);pa.CreateSelfCollisionAttr(True);pa.CreateParticleGroupAttr(0)
        PhysxSchema.PhysxParticleSetAPI.Apply(points.GetPrim()).CreateFluidAttr(True)
        UsdPhysics.MassAPI.Apply(points.GetPrim()).CreateMassAttr(ml*.001)
        # A lightly blue surface makes the otherwise colourless water legible in the task.
        visual=UsdShade.Material.Define(stage,'/World/Water/Visual');shader=UsdShade.Shader.Define(stage,'/World/Water/Visual/Surface');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.08,.32,.52));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.12);shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.)
        shader.CreateOutput('surface',Sdf.ValueTypeNames.Token);visual.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(visual)
        # Physics remains the same when surface extraction is unavailable.
        if hasattr(PhysxSchema,'PhysxParticleIsosurfaceAPI'):
            iso=PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(system.GetPrim());iso.CreateIsosurfaceEnabledAttr(True)
            if hasattr(iso,'CreateGridSpacingAttr'):iso.CreateGridSpacingAttr(d*.6)
            if hasattr(iso,'CreateMaxVerticesAttr'):iso.CreateMaxVerticesAttr(150000)
            if hasattr(iso,'CreateMaxTrianglesAttr'):iso.CreateMaxTrianglesAttr(300000)
            UsdShade.MaterialBindingAPI.Apply(system.GetPrim()).Bind(visual)
        self.points=points

    def positions(self):
        # simulationPoints avoids reading postprocessed rendering positions when available.
        p=self.points.GetPrim().GetAttribute('physxParticle:simulationPoints')
        v=p.Get() if p and p.HasAuthoredValueOpinion() else None
        return np.asarray(v if v is not None and len(v)==self.n else self.points.GetPositionsAttr().Get(),dtype=float)

    def inside(self,positions,T,c):
        v=(positions-T[:3,3])@T[:3,:3];z=v[:,2]
        r=c['radius_bottom']+(c['radius_top']-c['radius_bottom'])*z/c['height']-c['wall']+.0015
        return (z>=c['bottom']-.002)&(z<=c['height']+.002)&(np.sum(v[:,:2]**2,axis=1)<=r*r)

    def measure(self,source,receiver):
        p=self.positions()
        if len(p)!=self.n or not np.isfinite(p).all():raise RuntimeError('Invalid particle readback; cannot measure poured water.')
        a=self.inside(p,source,self.spec['objects']['source_cup']);b=self.inside(p,receiver,self.spec['objects']['receiver_cup'])
        spill=(~a)&(~b)&(p[:,2]<self.spec['table']['height_m']+.008)
        return {'source_ml':float(a.mean()*self.ml),'received_ml':float(b.mean()*self.ml),'spilled_ml':float(spill.mean()*self.ml),
                'in_flight_ml':float(((~a)&(~b)&(~spill)).mean()*self.ml),'particle_count':self.n,
                'readback_displacement_m':float(np.max(np.linalg.norm(p-self.initial,axis=1)))}
