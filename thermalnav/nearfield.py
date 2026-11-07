"""IntentNav Appendix A completion: inference is NOT free-space evidence.
Reference: https://arxiv.org/html/2606.08029v1#A1
"""
import math
import numpy as np
from .core import ray


def blind_zone(K,R,t,width,height,robot_xy,floor=0.):
    """Intersect actual bottom image rays with floor in world coordinates."""
    result=[]
    for u in (0.,K[0,2],float(width-1)):
        d=R@np.array([(u-K[0,2])/K[0,0],(height-1-K[1,2])/K[1,1],1.])
        if d[2]<-1e-6 and t[2]>floor:
            p=t+(floor-t[2])/d[2]*d
            result.append(float(np.linalg.norm(p[:2]-robot_xy)))
        else:result.append(None)
    return dict(camera_height_m=float(t[2]-floor),hfov_deg=float(np.degrees(np.arctan(K[0,2]/K[0,0])+np.arctan((width-1-K[0,2])/K[0,0]))),vfov_deg=float(np.degrees(np.arctan(K[1,2]/K[1,1])+np.arctan((height-1-K[1,2])/K[1,1]))),floor_blind_left_center_right_m=result)


def complete(bev,points,robot_xy,bin_degrees=3.,max_distance=2.,minimum_support=3):
    """points must be reliable floor/obstacle observations, never hole-filled.

    The nearest endpoint must itself be <=2m; far endpoints are not clipped
    into fictitious near-field evidence. Known obstacles terminate every ray.
    No sector filling between sparse rays and no change to occupancy/coverage.
    """
    pts=np.asarray(points)
    if pts.size==0:return {'valid_bins':0,'inferred_added':0}
    delta=pts[:,:2]-np.asarray(robot_xy);ranges=np.linalg.norm(delta,axis=1)
    valid=np.isfinite(pts).all(1)&(ranges>.05)&(ranges<=max_distance)
    pts=pts[valid];delta=delta[valid];ranges=ranges[valid]
    bins=np.floor((np.degrees(np.arctan2(delta[:,1],delta[:,0]))+180)/bin_degrees).astype(int)
    occ=bev.occupancy();before=np.count_nonzero(bev.inferred);count=0;src=bev.cell(robot_xy)
    for b in np.unique(bins):
        ids=np.where(bins==b)[0]
        if len(ids)<minimum_support:continue
        order=ids[np.argsort(ranges[ids])]
        # Require support near the nearest endpoint, not distant floor behind
        # a one-pixel leg. The leg still exists in the independent obstacle map.
        nearest=order[0]
        support=np.linalg.norm(pts[ids]-pts[nearest],axis=1)<=.12
        if np.count_nonzero(support)<minimum_support:continue
        dst=bev.cell(pts[nearest]);count+=1
        for x,y in ray(src,dst):
            if not bev.inside((x,y)) or occ[y,x]==100:break
            # Cell centre remains inside the 2m bound.
            if np.linalg.norm(np.asarray(bev.world((x,y)))-robot_xy)<=max_distance:
                bev.inferred[y,x]=True
    return {'valid_bins':count,'inferred_added':int(np.count_nonzero(bev.inferred)-before),'verified_free_promotions':int(np.count_nonzero(bev.occupancy()==0)-np.count_nonzero(occ==0))}
