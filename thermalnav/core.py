"""ROS-independent online mapping and conservative exploration primitives."""
from dataclasses import dataclass, field
from collections import deque
import math
import cv2
import numpy as np


def rotation(q):
    x,y,z,w=q
    n=math.sqrt(x*x+y*y+z*z+w*w)
    if n<1e-8: raise ValueError('invalid quaternion')
    x,y,z,w=np.array(q)/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def ray(a,b):
    """Integer Bresenham, including both endpoints."""
    x,y=map(int,a); ex,ey=map(int,b); dx=abs(ex-x); dy=-abs(ey-y)
    sx=1 if x<ex else -1; sy=1 if y<ey else -1; err=dx+dy
    while True:
        yield x,y
        if x==ex and y==ey: break
        e=2*err;oldx,oldy=x,y
        if e>=dy:err+=dy;x+=sx
        if e<=dx:err+=dx;y+=sy
        if x!=oldx and y!=oldy:
            # Conservative supercover at diagonal cell crossings.
            yield oldx,y
            yield x,oldy


@dataclass
class Candidate:
    id: str
    kind: str
    xy: tuple
    path: list
    gain: float
    cost: float
    score: float
    target_id: int = -1


class BEV:
    def __init__(self,resolution=.1,size=30.,origin=(-15.,-15.),footprint=.35):
        self.res=resolution; self.origin=np.array(origin,dtype=float)
        n=int(size/resolution); self.log=np.zeros((n,n),np.float32)
        self.seen=np.zeros((n,n),bool); self.scanned=np.zeros((n,n),bool)
        self.visits=np.zeros((n,n),np.uint16); self.footprint=footprint
        self.frames=0
        self.inferred=np.zeros((n,n),bool)
        self.certificate=None
        self.rejections={}
        self.nearfield_stats={}
        self.blind_zone={}
        self.floor_evidence=np.zeros((n,n),bool)
        self.diagnostics={}

    def cell(self,xy):return tuple(np.floor((np.asarray(xy)[:2]-self.origin)/self.res).astype(int))
    def world(self,p):return tuple(self.origin+(np.asarray(p)+.5)*self.res)
    def inside(self,p):return 0<=p[0]<self.log.shape[1] and 0<=p[1]<self.log.shape[0]
    def expand(self,points):
        cells=np.floor((np.asarray(points)[:,:2]-self.origin)/self.res).astype(int)
        lo=np.maximum(0,-cells.min(0)+10); hi=np.maximum(0,cells.max(0)+11-np.array(self.log.shape[::-1]))
        if not np.any(lo+hi):return
        pad=((int(lo[1]),int(hi[1])),(int(lo[0]),int(hi[0])))
        for k in ('log','seen','scanned','visits','inferred','floor_evidence'):setattr(self,k,np.pad(getattr(self,k),pad))
        self.origin-=lo*self.res

    def occupancy(self):
        out=np.full(self.log.shape,-1,np.int8)
        out[self.seen & (self.log<0)]=0
        out[self.seen & (self.log>=0)]=100
        return out

    def integrate(self,depth,K,R,t,floor=0.,stride=2,max_range=8.,robot_xy=None):
        """Only directly measured floor endpoints are verified free.

        Projecting a ray above the floor to 2D does not prove that a table leg
        below that ray is absent. Near-field ray completion has its own layer.
        Thin obstacles are retained at full pixel resolution, without a
        minimum cluster-size filter. No interpolation across depth edges.
        """
        if robot_xy is None:robot_xy=np.asarray(t[:2])
        v,u=np.mgrid[:depth.shape[0],:depth.shape[1]]
        valid=np.isfinite(depth)&(depth>=.15)&(depth<=max_range)
        # Integrate a regular pixel lattice. Validity statistics still report
        # the whole sensor frame, while ray work is bounded by ``stride``.
        ray_mask=valid&(u%stride==0)&(v%stride==0)
        z=depth[ray_mask];uu=u[ray_mask];vv=v[ray_mask]
        if not len(z):
            self.diagnostics={'depth_pixels':int(depth.size),'valid_depth':0,'world_points':0,'ground_points':0,'obstacle_points':0,'in_map_points':0}
            return
        points=np.column_stack(((uu-K[0,2])*z/K[0,0],(vv-K[1,2])*z/K[1,1],z))@R.T+t
        h=points[:,2]-floor
        obstacle=(h>.12)&(h<1.8)
        # Reliability: finite local neighbourhood and continuous measured
        # surface. This filter is applied to floor/completion, NOT obstacles.
        safe=np.where(valid,depth,np.nan)
        neighbours=np.stack([safe,np.roll(safe,1,0),np.roll(safe,-1,0),np.roll(safe,1,1),np.roll(safe,-1,1)])
        spread=np.max(neighbours,axis=0)-np.min(neighbours,axis=0)
        reliable=valid&np.isfinite(spread)&(spread<.08)
        reliable[[0,-1],:]=False;reliable[:,[0,-1]]=False
        stable=reliable[ray_mask]
        ground=(np.abs(h)<=.06)&stable
        self.diagnostics={'depth_pixels':int(depth.size),'valid_depth':int(valid.sum()),
            'world_points':int(len(points)),'ground_points':int(ground.sum()),
            'obstacle_points':int(obstacle.sum()),
            'world_min':points.min(axis=0).round(2).tolist(),
            'world_max':points.max(axis=0).round(2).tolist(),
            'height_p05_p50_p95':np.percentile(h,[5,50,95]).round(3).tolist(),
            'sensor_origin':np.asarray(t).round(3).tolist()}
        self.expand(np.vstack((points[ground|obstacle],t)))
        def cells(p):return np.floor((p[:,:2]-self.origin)/self.res).astype(int)
        fc=cells(points[ground]);oc=cells(points[obstacle])
        hit=np.zeros(self.log.shape,bool);free=np.zeros(self.log.shape,bool)
        if len(oc):hit[oc[:,1],oc[:,0]]=True
        bounds=(fc[:,0]>=0)&(fc[:,0]<self.log.shape[1])&(fc[:,1]>=0)&(fc[:,1]<self.log.shape[0])
        if len(oc):bounds_oc=(oc[:,0]>=0)&(oc[:,0]<self.log.shape[1])&(oc[:,1]>=0)&(oc[:,1]<self.log.shape[0])
        else:bounds_oc=np.zeros(0,bool)
        self.diagnostics['free_endpoints_in_map']=int(bounds.sum())
        self.diagnostics['obstacle_endpoints_in_map']=int(bounds_oc.sum())
        # Clear only camera rays that terminate on reliable ground. The old
        # map marked only endpoint cells free, shrinking a full camera view to
        # a few hundred cells. Stop at any known/current obstacle, and never
        # clear beyond the measured endpoint.
        sensor_cell=cells(np.asarray(t,dtype=float)[None,:])[0]
        for endpoint in fc[bounds]:
            for gx,gy in ray(sensor_cell,endpoint):
                if not self.inside((gx,gy)):
                    continue
                if hit[gy,gx] or (self.seen[gy,gx] and self.log[gy,gx]>=0):
                    break
                free[gy,gx]=True
        self.diagnostics['ray_cleared_cells']=int(free.sum())
        # Static-scene obstacles win even over floor visible under furniture.
        free &= ~hit & ~(self.seen & (self.log>=0))
        self.log[free]=np.maximum(-4,self.log[free]-.55);self.seen[free]=True;self.floor_evidence[free]=True
        self.log[hit]=np.minimum(5,np.maximum(0,self.log[hit])+1.4);self.seen[hit]=True;self.floor_evidence[hit]=False;self.inferred[hit]=False
        from .nearfield import complete,blind_zone
        self.nearfield_stats=complete(self,points[(ground|obstacle)&stable],np.asarray(robot_xy))
        self.blind_zone=blind_zone(K,R,t,depth.shape[1],depth.shape[0],np.asarray(robot_xy),floor)
        # Test actual floor centres against the depth buffer, including FOV.
        yy,xx=np.where(self.occupancy()==0)
        pts=np.column_stack((self.origin[0]+(xx+.5)*self.res,self.origin[1]+(yy+.5)*self.res,np.full(len(xx),floor)))
        cam=(pts-t)@R
        good=(cam[:,2]>.15)&(cam[:,2]<=max_range)
        idx=np.where(good)[0];c=cam[idx]
        uu=np.rint(K[0,0]*c[:,0]/c[:,2]+K[0,2]).astype(int)
        vv=np.rint(K[1,1]*c[:,1]/c[:,2]+K[1,2]).astype(int)
        inside=(uu>=1)&(uu<depth.shape[1]-1)&(vv>=1)&(vv<depth.shape[0]-1)
        idx=idx[inside];uu=uu[inside];vv=vv[inside]
        # Minimum neighbourhood depth prevents interpolation across occluders.
        if len(idx):
            ds=np.stack([depth[vv+dy,uu+dx] for dx,dy in ((0,0),(1,0),(-1,0),(0,1),(0,-1))])
            measured=np.min(ds,axis=0)
            visible=np.isfinite(measured)&(measured>.15)&(cam[idx,2]<=measured+.04)
            self.scanned[yy[idx[visible]],xx[idx[visible]]]=True
        self.frames+=1
        occ=self.occupancy()
        self.diagnostics.update(observed_cells=int(self.seen.sum()),free_cells=int((occ==0).sum()),
            occupied_cells=int((occ==100).sum()),scanned_cells=int(self.scanned.sum()),
            occupancy_checksum=__import__('hashlib').md5(occ.tobytes()).hexdigest())

    def coverage(self,targets=()):
        occ=self.occupancy();out=np.full(occ.shape,-1,np.int8)
        out[occ==0]=0;out[(occ==0)&self.scanned]=50;out[occ==100]=100
        for t in targets:
            x,y=self.cell(t['xy'])
            if self.inside((x,y)):out[y,x]=75
        return out

    def accept_certificate(self,proof,robot_xy,stamp):
        """Accept only bounded, fresh simulator collision-query attestation.

        No caller flag such as assume_safe is supported. A certificate never
        writes seen/log/floor_evidence and never moves along with the robot.
        """
        self.certificate_rejection=''
        def reject(reason):self.certificate_rejection=reason;return False
        if self.certificate is not None:return reject('already_certified')
        try:
            if proof['provider']!='thermalnav_physx_startup_v1' or proof['frame']!='odom':return reject('wrong_provider_or_frame')
            if not all(proof.get(k) is True for k in ('collision_free','ground_supported','initial_pose')):return reject('failed_physics_validation')
            xy=np.asarray(proof['xy'],dtype=float);r=float(proof['radius']);age=stamp-float(proof['stamp'])
            if xy.shape!=(2,) or not np.isfinite(xy).all() or not 0<r<=self.footprint:return reject('invalid_radius_or_pose')
            if not -1e-6<=age<=3:return reject('stale_stamp')
            if np.linalg.norm(xy-robot_xy)>.04:return reject('mismatched_pose')
            if not proof.get('sim_pid') or not proof.get('query_count',0)>=9:return reject('missing_validation_queries')
            self.certificate=dict(proof)
            if self.footprint_reason(robot_xy)=='occupied':self.certificate=None;return reject('observed_obstacle_conflict')
            return True
        except (KeyError,TypeError,ValueError):return reject('malformed_proof')

    def footprint_reason(self,xy):
        """Continuous circular footprint against intersecting grid squares.

        Unknown squares need direct free evidence or coverage by the fixed
        certificate. Inference is never accepted as traversable evidence.
        """
        xy=np.asarray(xy);r=self.footprint;occ=self.occupancy()
        lo=np.floor((xy-r-self.origin)/self.res).astype(int);hi=np.floor((xy+r-self.origin)/self.res).astype(int)
        cert=self.certificate
        covered=cert is not None and np.linalg.norm(xy-np.asarray(cert['xy']))+r<=cert['radius']+1e-9
        unknown=False
        for y in range(lo[1],hi[1]+1):
            for x in range(lo[0],hi[0]+1):
                lower=self.origin+np.array([x,y])*self.res;upper=lower+self.res
                closest=np.clip(xy,lower,upper)
                if np.linalg.norm(closest-xy)>=r-1e-9:continue
                if not self.inside((x,y)):unknown=True;continue
                if occ[y,x]==100:return 'occupied'
                if occ[y,x]==0:continue
                if covered:continue
                if cert is not None:
                    corners=np.array([lower,upper,[lower[0],upper[1]],[upper[0],lower[1]]])
                    if np.max(np.linalg.norm(corners-np.asarray(cert['xy']),axis=1))<=cert['radius']:continue
                unknown=True
        return 'unknown_footprint' if unknown else None

    def path_reason(self,path):
        if not path:return 'no_safe_path'
        for a,b in zip(path,path[1:]):
            a=np.asarray(a);b=np.asarray(b)
            for f in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(b-a)/(self.res/2)))+1)):
                reason=self.footprint_reason(a+(b-a)*f)
                if reason:return reason
        return self.footprint_reason(path[-1])

    def reachable(self,robot):
        occ=self.occupancy();r=int(math.ceil(self.footprint/self.res))
        kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*r+1,2*r+1))
        blocked=cv2.dilate((occ!=0).astype(np.uint8),kernel)>0
        start=self.cell(robot);reason=self.footprint_reason(robot)
        self.startup_check=reason or 'safe'
        if reason or not self.inside(start):return {},{},blocked
        # A certificate permits the actual starting pose, not arbitrary free
        # cells. Every outgoing edge is checked over its swept footprint.
        dist={start:0.};parent={start:None};q=deque([start])
        while q:
            a=q.popleft();aw=robot if a==start else self.world(a)
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                b=(a[0]+dx,a[1]+dy)
                if not self.inside(b) or b in dist or occ[b[1],b[0]]!=0:continue
                if blocked[b[1],b[0]] and self.footprint_reason(self.world(b)):continue
                if a==start and self.path_reason([aw,self.world(b)]):continue
                # For grid-aligned adjacent centres the conservative inflation
                # is a superset of the swept footprint. Border edges use exact checks.
                if (blocked[a[1],a[0]] or blocked[b[1],b[0]]) and self.path_reason([aw,self.world(b)]):continue
                dist[b]=dist[a]+self.res;parent[b]=a;q.append(b)
        return dist,parent,blocked

    def candidates(self,robot,failed=(),targets=()):
        occ=self.occupancy();dist,parent,blocked=self.reachable(robot)
        self.rejections={k:0 for k in ('unknown_footprint','occupied','disconnected','no_safe_path','too_small','repeat_visit')}
        unknown=(occ==-1).astype(np.uint8)
        boundary=(occ==0)&(cv2.dilate(unknown,np.ones((3,3),np.uint8))>0)
        n,labels,stats,centres=cv2.connectedComponentsWithStats(boundary.astype(np.uint8),8)
        self.frontier_components=n-1
        if not dist:
            self.rejections[self.startup_check if self.startup_check in self.rejections else 'no_safe_path']=n-1
            return []
        reachable=np.array(list(dist));out=[]
        def add(kind,anchor,gain,target_id=-1):
            ds=np.linalg.norm(reachable-np.asarray(anchor),axis=1)
            order=np.argsort(ds)
            for j in order[:120]:
                cell=tuple(reachable[j]);xy=self.world(cell);cost=dist[cell]
                if cost<.85 or any(np.linalg.norm(np.array(xy)-f)<1.0 for f in failed):continue
                if ds[j]*self.res>1.8:self.rejections['disconnected']+=1;return
                path=[];p=cell
                while p is not None:path.append(self.world(p));p=parent[p]
                path.reverse();path[0]=tuple(robot)
                reason=self.path_reason(path)
                if reason:self.rejections[reason]+=1;continue
                penalty=float(self.visits[cell[1],cell[0]])
                score=gain/(1+cost)-2*penalty+(8 if kind=='verify_target' else 0)
                out.append(Candidate(f'{kind}_{len(out)}',kind,xy,path,float(gain),cost,score,target_id));return
            self.rejections['no_safe_path']+=1
        for lab in range(1,n):
            if stats[lab,cv2.CC_STAT_AREA]<4:self.rejections['too_small']+=1;continue
            anchor=centres[lab];cx,cy=map(int,anchor);rad=15
            gain=np.count_nonzero(unknown[max(0,cy-rad):cy+rad+1,max(0,cx-rad):cx+rad+1])*.01
            add('frontier',anchor,gain)
        unseen=((occ==0)&~self.scanned).astype(np.uint8)
        n,labels,stats,centres=cv2.connectedComponentsWithStats(unseen,8)
        for lab in range(1,n):
            if stats[lab,cv2.CC_STAT_AREA]>=12:add('inspection_viewpoint',centres[lab],stats[lab,cv2.CC_STAT_AREA]*.01)
        for t in targets:
            if not t.get('verified',False):add('verify_target',self.cell(t['xy']),4.,t['id'])
        return sorted(out,key=lambda c:c.score,reverse=True)[:24]


class Tracks:
    def __init__(self):self.items=[]
    def add(self,xy,stamp,label='person'):
        for t in self.items:
            if t['label']==label and np.linalg.norm(np.array(t['xy'])-xy)<.8:
                if stamp-t['last']<.2:return t
                t['xy']=tuple(.7*np.array(t['xy'])+.3*np.array(xy));t['hits']+=1;t['last']=stamp
                t['verified']=t['hits']>=3 and stamp-t['first']>=1.
                return t
        t=dict(id=len(self.items),xy=tuple(xy),label=label,hits=1,first=stamp,last=stamp,verified=False)
        self.items.append(t);return t


def localize_box(box,depth,K,R,t):
    x1,y1,x2,y2=box;w=x2-x1;h=y2-y1
    xa=max(0,int(x1+.3*w));xb=min(depth.shape[1],int(x2-.3*w))
    ya=max(0,int(y1+.3*h));yb=min(depth.shape[0],int(y2-.3*h))
    patch=depth[ya:yb,xa:xb];vals=patch[np.isfinite(patch)&(patch>.15)&(patch<8)]
    if len(vals)<6:return None
    z=float(np.median(vals));u=(xa+xb)/2;v=(ya+yb)/2
    return R@np.array([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z])+t
