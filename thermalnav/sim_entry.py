"""Optional Isaac entry: original simulation + one bounded startup attestation.

No navigation maps or target ground truth are exposed. Only the initial
footprint is queried; all subsequent planning uses camera evidence.
"""
import os,math,json
from core import omniverse_sim as sim
from omni.physx import get_physx_scene_query_interface
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy
import numpy as np

# Headless IsaacLab disables scene queries by default. Enable the query
# acceleration structure before physics creation; this does not alter dynamics.
original_make=sim.gym.make
def make_with_scene_queries(*args,**kwargs):
    kwargs['cfg'].sim.enable_scene_query_support=True
    return original_make(*args,**kwargs)
sim.gym.make=make_with_scene_queries
original=sim.pub_robo_data_ros2
publisher=None
initial=None
finished=False

def publish_with_certificate(*args,**kwargs):
    global publisher,initial,finished
    original(*args,**kwargs)
    if finished:return
    base=args[2];env=args[3];robot=env.unwrapped.scene['robot']
    pos=robot.data.root_pos_w[0].detach().cpu().numpy();stamp=float(kwargs['timestamp_seconds'])
    if initial is None:initial=pos.copy()
    if stamp<1.:return
    if stamp>8.:finished=True;print('[ThermalNav] startup certificate unavailable: robot did not settle',flush=True);return
    speed=float(robot.data.root_lin_vel_b[0].norm())
    if speed>.04:return
    if np.linalg.norm(pos[:2]-initial[:2])>.10:
        finished=True;print('[ThermalNav] startup certificate rejected: moved from spawn',flush=True);return
    radius=.35;query=get_physx_scene_query_interface();grounds=[];queries=0
    # Only current-footprint probes, no environmental GT map construction.
    for angle in [None]+list(np.linspace(0,2*math.pi,8,endpoint=False)):
        x,y=pos[:2] if angle is None else pos[:2]+radius*np.array([math.cos(angle),math.sin(angle)])
        hits=[]
        def floor_hit(hit):
            path=str(getattr(hit,'collision',''));p=getattr(hit,'position',None);normal=getattr(hit,'normal',None)
            if '/Robot/' not in path and p is not None and normal is not None and normal.z>.95 and -.10<float(p.z)<.10:hits.append(float(p.z))
            return True
        query.raycast_all(sim.carb.Float3(float(x),float(y),float(pos[2]+.1)),sim.carb.Float3(0,0,-1),1.,floor_hit);queries+=1
        if hits:grounds.append(max(hits))
    supported=len(grounds)==9 and max(grounds)-min(grounds)<.04
    blocked=[]
    def collision_hit(hit):
        path=str(getattr(hit,'collision','') or getattr(hit,'rigid_body',''))
        if '/Robot/' not in path and not path.endswith('/Robot'):blocked.append(path)
        return True
    floor=float(np.median(grounds)) if grounds else 0.
    # Original mapping obstacle-height band [.12,1.8]; .35 footprint unchanged.
    query.overlap_box(sim.carb.Float3(radius,radius,.84),sim.carb.Float3(float(pos[0]),float(pos[1]),floor+.96),sim.carb.Float4(0,0,0,1),collision_hit,False);queries+=1
    print('[ThermalNav] startup floor support samples='+str(grounds),flush=True)
    proof=dict(provider='thermalnav_physx_startup_v1',frame='odom',stamp=stamp,xy=pos[:2].tolist(),radius=radius,collision_free=not blocked,ground_supported=bool(supported),initial_pose=True,sim_pid=os.getpid(),query_count=queries)
    publisher=base.create_publisher(String,'/thermalnav/startup_certificate',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(String(data=json.dumps(proof)));finished=True
    print('[ThermalNav] startup footprint attestation '+json.dumps(proof)+' blocked='+str(blocked[:3]),flush=True)

sim.pub_robo_data_ros2=publish_with_certificate
sim.run_sim()
