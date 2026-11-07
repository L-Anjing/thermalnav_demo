import time,json,sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image,CameraInfo,Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Twist
from tf2_ros import Buffer,TransformListener
from cv_bridge import CvBridge

def startup_audit():
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from .core import BEV
    rclpy.init();n=Node('thermalnav_startup_audit');data={};subs=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for name,topic,kind in [('map','/thermalnav/bev',OccupancyGrid),('proof','/thermalnav/startup_certificate',String)]:
        subs.append(n.create_subscription(kind,topic,lambda m,k=name:data.update({k:m}),qos))
    end=time.monotonic()+8
    while time.monotonic()<end and len(data)<2:rclpy.spin_once(n,timeout_sec=.1)
    if len(data)==2:
        m=data['map'];proof=json.loads(data['proof'].data);b=BEV();occ=np.array(m.data).reshape(m.info.height,m.info.width)
        b.log=np.where(occ==0,-2.,2.);b.seen=occ!=-1;b.origin=np.array([m.info.origin.position.x,m.info.origin.position.y]);b.res=m.info.resolution
        ok=b.accept_certificate(proof,np.array(proof['xy']),proof['stamp']+.01)
        yy,xx=np.where(occ==100);points=np.column_stack((b.origin[0]+(xx+.5)*b.res,b.origin[1]+(yy+.5)*b.res))
        nearby=points[np.linalg.norm(points-np.array(proof['xy']),axis=1)<.6]
        print('startup proof:',proof,'accepted_at_attested_pose=',ok,'reason=',getattr(b,'certificate_rejection',''),'nearby_occupied=',nearby.tolist())
    else:print('FAIL missing startup audit topics:',list(data))
    n.destroy_node();rclpy.shutdown()

def main():
    if '--startup' in sys.argv:return startup_audit()
    rclpy.init();n=Node('thermalnav_doctor');buf=Buffer();listener=TransformListener(buf,n);got={};counts={};subs=[]
    topics={'/robot0/front_cam/rgb':Image,'/robot0/front_cam/depth':Image,'/robot0/front_cam/camera_info':CameraInfo,'/robot0/odom':Odometry,'/robot0/imu':Imu}
    def cb(m,t):got[t]=m;counts[t]=counts.get(t,0)+1
    for t,k in topics.items():subs.append(n.create_subscription(k,t,lambda m,t=t:cb(m,t),10))
    end=time.monotonic()+10
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    ok=True
    for t in topics:
        present=t in got and counts[t]>1;ok &=present
        print(('PASS' if present else 'FAIL'),t,topics[t].__name__,'samples='+str(counts.get(t,0)))
    d=got.get('/robot0/front_cam/depth');o=got.get('/robot0/odom');i=got.get('/robot0/front_cam/camera_info')
    if d is not None:
        a=CvBridge().imgmsg_to_cv2(d,'passthrough');v=a[np.isfinite(a)&(a>0)]
        print('PASS' if d.encoding in ('32FC1','16UC1') else 'FAIL','Depth',d.encoding,f'{d.width}x{d.height}','range='+str((float(v.min()),float(v.max()))) if len(v) else 'NO VALID DEPTH')
        try:tf=buf.lookup_transform('odom',d.header.frame_id,Time.from_msg(d.header.stamp));print('PASS stamped TF odom ->',d.header.frame_id)
        except Exception as e:ok=False;print('FAIL stamped TF',type(e).__name__)
    if d is not None and o is not None:
        diff=abs((d.header.stamp.sec-o.header.stamp.sec)+(d.header.stamp.nanosec-o.header.stamp.nanosec)*1e-9);ok &=diff<.3
        print('PASS' if diff<.3 else 'FAIL','depth/odom time skew',round(diff,4))
    if i is not None:print('CameraInfo K=',list(i.k))
    names=dict(n.get_topic_names_and_types());print('PASS /clock published' if '/clock' in names else 'PASS /clock absent: use_sim_time=false; stamped TF uses sensor time')
    consumers=n.get_subscriptions_info_by_topic('/goal_pose');controllers=[x for x in consumers if 'navigation' in x.node_name or 'rl_nav' in x.node_name]
    ok &= bool(controllers);print('PASS' if controllers else 'FAIL','SRU /goal_pose PoseStamped subscribers:',[x.node_name for x in consumers])
    pubs=n.get_publishers_info_by_topic('/robot0/cmd_vel');ok &=len(pubs)==1
    print('PASS' if len(pubs)==1 else 'FAIL','exclusive cmd_vel publisher:',[x.node_name for x in pubs])
    print('NOT TESTED navigation motion/arrival (doctor is read-only)')
    n.destroy_node();rclpy.shutdown();raise SystemExit(0 if ok else 1)
if __name__=='__main__':main()
