"""ROS integration: one high-level goal, original SRU owns every Twist."""
import argparse,json,math,time,signal,os
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry,OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped,Point,Twist
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException
from cv_bridge import CvBridge
import cv2
from .renderer import BEVRenderer
from .core import BEV,Tracks,rotation,localize_box
from .detector import Detector
from .policy import Policy

class Demo(Node):
    def __init__(self,mode='heuristic',observe=False):
        super().__init__('thermalnav_demo')
        self.bev=BEV();self.renderer=BEVRenderer();self.camera_R=None;self.camera_t=None;self.tracks=Tracks();self.policy=Policy(mode);self.detector=Detector();self.bridge=CvBridge()
        self.tf=Buffer(cache_time=Duration(seconds=20));self.listener=TransformListener(self.tf,self)
        self.K=None;self.info=None;self.odom=None;self.rgb=None;self.rgbs=deque(maxlen=15);self.pending=deque(maxlen=3)
        self.last_stamp=-1.;self.last_data=0.;self.last_rgb_detect=0.;self.last_map_wall=0.
        self.reset_waiting_odom=False;self.last_depth_stats={};self.tf_failures=0
        self.state='EXPLORE';self.enabled=not observe;self.active=None;self.failed=[];self.history=[]
        self.selected=0;self.completed=0;self.completed_kinds={'frontier':0,'inspection_viewpoint':0,'verify_target':0};self.reached=[];self.certificate_status='unavailable';self.proof=None;self.nav_failures=0;self.sent=None;self.started=0.;self.last_progress=0.;self.best=1e9
        self.cancel_until=0.;self.cmd_speed=0.;self.cmd_time=0.;self.last_goal_wall=0.;self.last_decision=0.
        self.pool=ThreadPoolExecutor(max_workers=1);self.future=None;self.future_candidates=[]
        self.trajectory=Path();self.trajectory.header.frame_id='odom';self.candidates=[];self.blocked='waiting for sensors'
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(OccupancyGrid,'/thermalnav/bev',qos)
        self.cover_pub=self.create_publisher(OccupancyGrid,'/thermalnav/coverage',qos)
        self.explored_pub=self.create_publisher(OccupancyGrid,'/thermalnav/explored',qos)
        self.inferred_pub=self.create_publisher(OccupancyGrid,'/thermalnav/inferred',qos)
        self.render_pub=self.create_publisher(Image,'/thermalnav/bev/rendered',qos)
        self.cover_render_pub=self.create_publisher(Image,'/thermalnav/bev/coverage_rendered',qos)
        self.create_subscription(String,'/thermalnav/startup_certificate',self.on_certificate,qos)
        self.marker_pub=self.create_publisher(MarkerArray,'/thermalnav/markers',qos)
        self.path_pub=self.create_publisher(Path,'/thermalnav/trajectory',qos)
        self.status_pub=self.create_publisher(String,'/thermalnav/status',qos)
        self.goal_pub=self.create_publisher(PoseStamped,'/goal_pose',1)
        self.create_subscription(CameraInfo,'/robot0/front_cam/camera_info',self.on_info,qos_profile_sensor_data)
        self.create_subscription(Image,'/robot0/front_cam/depth',self.on_depth,10)
        self.create_subscription(Image,'/robot0/front_cam/rgb',self.on_rgb,5)
        self.create_subscription(Odometry,'/robot0/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Twist,'/robot0/cmd_vel',self.on_cmd,qos_profile_sensor_data)
        self.create_service(Trigger,'/thermalnav/start',self.start)
        self.create_service(Trigger,'/thermalnav/stop',self.stop)
        self.create_timer(.1,self.process);self.create_timer(1.,self.tick)
        self.get_logger().info('Detector: '+self.detector.name+'; '+self.policy.status)

    def on_certificate(self,m):
        try:self.proof=json.loads(m.data)
        except ValueError:self.certificate_status='rejected: malformed attestation'
    @staticmethod
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def start(self,req,res):self.enabled=True;res.success=True;res.message='Exploration enabled';return res
    def stop(self,req,res):self.enabled=False;self.cancel();res.success=True;res.message='Current-pose PointGoal sent; wait for SRU zero command';return res
    def on_info(self,m):
        if m.header.frame_id!='robot0/front_cam' or m.k[0]<=0 or m.k[4]<=0:return
        self.info=m;self.K=np.array(m.k).reshape(3,3)
    def on_rgb(self,m):
        try:self.rgb=self.bridge.imgmsg_to_cv2(m,'bgr8');self.rgbs.append((self.stamp(m),self.rgb,m.header.frame_id))
        except Exception as e:self.blocked='RGB decode: '+str(e)
    def on_depth(self,m):
        if self.stamp(m)<self.last_stamp-.5:
            # Isaac episode resets rewind sim time but keep the same static
            # odom/world frame. Preserve valid global map evidence; cancel the
            # stale PointGoal only after the first odom from the new epoch.
            self.pending.clear();self.reset_waiting_odom=True
            self.active=None;self.sent=None;self.cancel_until=time.monotonic()+1.
            self.last_stamp=-1.;self.proof=None;self.bev.certificate=None
            self.tracks=Tracks();self.blocked='simulation time epoch reset; waiting for fresh odom'
            self.get_logger().warn('Simulation time rewound; keeping the static-world global BEV and refreshing the SRU target.')
        self.pending.append(m)
    def on_odom(self,m):
        if m.header.frame_id!='odom':self.blocked='odom frame mismatch';return
        self.odom=m;self.last_data=time.monotonic()
        if self.reset_waiting_odom:
            self.reset_waiting_odom=False
            p=PoseStamped();p.header=m.header;p.pose=m.pose.pose;self.goal_pub.publish(p)
            self.cancel_until=time.monotonic()+1.;self.blocked=''
        p=PoseStamped();p.header=m.header;p.pose=m.pose.pose
        if not self.trajectory.poses or np.linalg.norm(self.xy()-np.array([self.trajectory.poses[-1].pose.position.x,self.trajectory.poses[-1].pose.position.y]))>.08:
            self.trajectory.poses.append(p)
            x,y=self.bev.cell(self.xy())
            if self.bev.inside((x,y)):self.bev.visits[y,x]=min(65535,int(self.bev.visits[y,x])+1)
    def on_cmd(self,m):self.cmd_speed=abs(m.linear.x)+abs(m.linear.y)+abs(m.angular.z);self.cmd_time=time.monotonic()
    def xy(self):return np.array([self.odom.pose.pose.position.x,self.odom.pose.pose.position.y])
    def transform(self,frame,stamp):
        tf=self.tf.lookup_transform('odom',frame,Time.from_msg(stamp))
        q=tf.transform.rotation;t=tf.transform.translation
        return rotation([q.x,q.y,q.z,q.w]),np.array([t.x,t.y,t.z])
    def process(self):
        if not self.pending or self.K is None or self.odom is None:return
        m=self.pending[-1]
        if m.header.frame_id!=self.info.header.frame_id or (m.width,m.height)!=(self.info.width,self.info.height):self.blocked='depth/info mismatch';return
        skew=abs(self.stamp(m)-self.stamp(self.odom))
        if skew>.25:self.blocked='depth/odom time mismatch';return
        try:R,t=self.transform(m.header.frame_id,m.header.stamp)
        except TransformException:self.tf_failures+=1;self.blocked='waiting for stamped camera TF';return
        if self.blocked.startswith('waiting for stamped camera TF') or self.blocked.startswith('depth/odom time mismatch') or self.blocked.startswith('simulation time epoch reset'):
            self.blocked=''
        self.pending.clear()
        if m.encoding not in ('32FC1','16UC1'):self.blocked='unsupported depth encoding '+m.encoding;return
        depth=self.bridge.imgmsg_to_cv2(m,'passthrough').astype(np.float32)
        if m.encoding=='16UC1':depth*=.001
        self.camera_R=R.copy();self.camera_t=t.copy()
        self.bev.integrate(depth,self.K,R,t,robot_xy=self.xy())
        valid=depth[np.isfinite(depth)&(depth>=.15)&(depth<=8.)]
        self.last_depth_stats={'stamp':self.stamp(m),'encoding':m.encoding,'shape':[m.height,m.width],
            'valid_pixels':int(valid.size),'invalid_pixels':int(depth.size-valid.size),
            'depth_min_m':float(valid.min()) if valid.size else None,'depth_median_m':float(np.median(valid)) if valid.size else None,
            'depth_max_m':float(valid.max()) if valid.size else None,'odom_stamp_skew_s':skew,
            'camera_pose_odom_m':t.round(3).tolist(),'camera_pose_changes_with_robot':True,
            'rgb_depth_skew_s':min((abs(x[0]-self.stamp(m)) for x in self.rgbs if x[2]==m.header.frame_id),default=None)}
        if self.proof is not None and self.bev.certificate is None:
            accepted=self.bev.accept_certificate(self.proof,self.xy(),self.stamp(m))
            self.certificate_status='certified' if accepted else 'rejected: '+self.bev.certificate_rejection
            if not accepted and self.stamp(m)-float(self.proof.get('stamp',0))>3:self.proof=None
        self.last_stamp=self.stamp(m);self.last_map_wall=time.monotonic()
        if self.rgbs and time.monotonic()-self.last_rgb_detect>2:
            stamp,rgb,frame=min(self.rgbs,key=lambda x:abs(x[0]-self.last_stamp))
            if abs(stamp-self.last_stamp)<.08 and frame==m.header.frame_id and rgb.shape[:2]==depth.shape:
                self.last_rgb_detect=time.monotonic()
                try:
                    for box,score,label in self.detector.detect(rgb):
                        p=localize_box(box,depth,self.K,R,t)
                        if p is not None:self.tracks.add(p[:2],stamp,label)
                except Exception as e:self.get_logger().warn('Detection unavailable: '+str(e),throttle_duration_sec=10)

    def goal(self,xy):
        p=PoseStamped();p.header=self.odom.header;p.pose.position.x=float(xy[0]);p.pose.position.y=float(xy[1]);p.pose.position.z=float(self.odom.pose.pose.position.z);p.pose.orientation.w=1.
        self.goal_pub.publish(p);self.sent=np.array(xy);self.last_goal_wall=time.monotonic();self.best=1e9;self.last_progress=self.last_stamp
    def cancel(self):
        if self.odom is not None:self.goal(self.xy())
        self.active=None;self.cancel_until=time.monotonic()+1.
    def choose_waypoint(self,c):
        xy=self.xy();near=int(np.argmin([np.linalg.norm(np.array(p)-xy) for p in c.path]))
        chosen=None
        for p in c.path[near+1:]:
            distance=np.linalg.norm(np.array(p)-xy)
            if distance>1.6:break
            if self.bev.path_reason([xy,p]):break
            if distance>.85:chosen=p
        if chosen is None:
            self.failed.append(c.xy);self.nav_failures+=1;self.blocked='no_safe_path: no straight SRU segment beyond arrival radius';self.cancel();return
        self.goal(chosen)
    def tick(self):
        now=time.monotonic()
        if self.odom is None:return
        if self.enabled and (now-self.last_data>3 or now-self.last_map_wall>5):
            if self.active:self.cancel()
            self.blocked='stale sensors: navigation held';self.publish();return
        if self.enabled and now>=self.cancel_until:
            if self.active:
                dist=float(np.linalg.norm(self.sent-self.xy()))
                if dist<self.best-.08:self.best=dist;self.last_progress=self.last_stamp
                if dist<=.75 and self.cmd_speed<.03 and now-self.cmd_time<2 and now-self.last_goal_wall>1:
                    if np.linalg.norm(np.array(self.active.xy)-self.xy())<=.8:
                        self.completed+=1;self.completed_kinds[self.active.kind]+=1;self.reached.append(self.active.xy);self.history.append({'id':self.active.id,'result':'reached','kind':self.active.kind,'xy':self.active.xy})
                        x,y=self.bev.cell(self.active.xy);self.bev.visits[y,x]+=5
                        self.active=None;self.sent=None;self.cancel_until=now+1
                    else:self.choose_waypoint(self.active)
                elif self.last_stamp-self.last_progress>15 or self.last_stamp-self.started>90:
                    self.failed.append(self.active.xy);self.history.append({'id':self.active.id,'result':'timeout','xy':self.active.xy});self.nav_failures+=1;self.cancel()
                else:
                    # Recheck the active route against the newest occupancy.
                    occ=self.bev.occupancy();cell=self.bev.cell(self.sent)
                    if not self.bev.inside(cell) or occ[cell[1],cell[0]]!=0 or self.bev.path_reason([self.xy(),self.sent]):self.failed.append(self.active.xy);self.nav_failures+=1;self.cancel()
            elif self.future is not None:
                if self.future.done():
                    c=self.future.result();self.future=None
                    # Revalidate VLM result on current map before sending.
                    fresh=self.bev.candidates(self.xy(),self.failed+self.reached,self.tracks.items)
                    match=next((x for x in fresh if c and x.kind==c.kind and np.linalg.norm(np.array(x.xy)-c.xy)<.3),None)
                    if match:self.activate(match)
            elif now-self.last_decision>3 and self.cmd_speed<.03 and now-self.cmd_time<2:
                self.last_decision=now;self.candidates=self.bev.candidates(self.xy(),self.failed+self.reached,self.tracks.items)
                if self.candidates:
                    if self.policy.mode=='vlm' and self.policy.url and self.policy.model:
                        import copy
                        self.future=self.pool.submit(self.policy.choose,self.candidates,copy.deepcopy(self.bev),None if self.rgb is None else self.rgb.copy(),list(self.history),list(self.tracks.items))
                    else:self.activate(self.policy.choose(self.candidates,self.bev,self.rgb,self.history,self.tracks.items))
                else:
                    occ=self.bev.occupancy();remaining=int(np.count_nonzero((occ==0)&~self.bev.scanned))
                    # No frontier alone is never completion. Unknown boundaries,
                    # unreachable free space, unverified tracks => explicit block.
                    boundary=np.any((occ==0)&( __import__('cv2').dilate((occ==-1).astype(np.uint8),np.ones((3,3),np.uint8))>0))
                    if remaining==0 and not boundary and not any(not t['verified'] for t in self.tracks.items) and self.bev.frames>0:self.state='FINISH';self.enabled=False
                    else:self.blocked=f'no safe reachable candidate; startup={self.bev.startup_check}; rejected={self.bev.rejections}; unscanned={remaining}; frontier_boundary={bool(boundary)}'
        self.publish()
    def activate(self,c):
        self.blocked='';self.active=c;self.selected+=1;self.started=self.last_stamp
        self.state={'frontier':'EXPLORE','inspection_viewpoint':'INSPECT','verify_target':'VERIFY_TARGET'}[c.kind]
        self.choose_waypoint(c);self.get_logger().info(f'{self.state} {c.id} xy={c.xy} gain={c.gain:.2f}')
    def grid(self,data):
        m=OccupancyGrid();m.header=self.odom.header;m.info.resolution=self.bev.res;m.info.width=data.shape[1];m.info.height=data.shape[0]
        m.info.origin.position.x=float(self.bev.origin[0]);m.info.origin.position.y=float(self.bev.origin[1]);m.info.origin.orientation.w=1.;m.data=data.ravel().tolist();return m
    def publish(self):
        occ=self.bev.occupancy();self.map_pub.publish(self.grid(occ));self.cover_pub.publish(self.grid(self.bev.coverage(self.tracks.items)))
        self.explored_pub.publish(self.grid(np.where(self.bev.seen,100,np.where(self.bev.inferred,50,-1)).astype(np.int8)))
        self.inferred_pub.publish(self.grid(np.where(self.bev.inferred,50,-1).astype(np.int8)))
        self.trajectory.header=self.odom.header;self.path_pub.publish(self.trajectory)
        q=self.odom.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        image=self.renderer.render(self.bev,self.xy(),yaw,[ [p.pose.position.x,p.pose.position.y] for p in self.trajectory.poses],self.candidates,self.sent,self.tracks.items)
        cov_image=self.renderer.coverage(self.bev,self.xy())
        for pub,array in ((self.render_pub,image),(self.cover_render_pub,cov_image)):
            msg=self.bridge.cv2_to_imgmsg(array,encoding='rgb8');msg.header.stamp=self.odom.header.stamp;msg.header.frame_id='odom';pub.publish(msg)
        arr=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;arr.markers.append(clear)
        def marker(xy,ns,i,color,text=''):
            m=Marker();m.header=self.odom.header;m.ns=ns;m.id=i;m.type=Marker.TEXT_VIEW_FACING if text else Marker.SPHERE;m.action=Marker.ADD
            m.pose.position.x=float(xy[0]);m.pose.position.y=float(xy[1]);m.pose.position.z=.7 if text else .15;m.pose.orientation.w=1.
            m.scale.x=.25;m.scale.y=.25;m.scale.z=.25;m.color.r,m.color.g,m.color.b=color;m.color.a=1.;m.text=text;arr.markers.append(m)
        for i,c in enumerate(self.candidates):marker(c.xy,'candidates',i,(.2,.8,1.))
        if self.sent is not None:marker(self.sent,'selected',0,(1.,.7,0.))
        for t in self.tracks.items:marker(t['xy'],'targets',t['id'],(1.,0.,0.),t['label']+(' verified' if t['verified'] else ' suspect'))
        marker(self.xy(),'policy',0,(1.,1.,1.),self.state+' '+self.policy.status)
        self.marker_pub.publish(arr)
        total=int(np.count_nonzero(occ==0));scanned=int(np.count_nonzero((occ==0)&self.bev.scanned))
        status=dict(state=self.state,enabled=self.enabled,policy=self.policy.status,pose_source='sim_gt_pose_via_odom',map_frame='odom',robot_xy=self.xy().round(3).tolist(),trajectory_poses=len(self.trajectory.poses),tf_failures=self.tf_failures,last_depth=self.last_depth_stats,bev=self.bev.diagnostics,occupancy_grid_checksum=__import__('hashlib').md5(occ.tobytes()).hexdigest(),coverage_grid_checksum=__import__('hashlib').md5(self.bev.coverage(self.tracks.items).tobytes()).hexdigest(),map_frames=self.bev.frames,selected=self.selected,completed=self.completed,completed_by_kind=self.completed_kinds,certificate=self.certificate_status,certificate_initial_check=(self.bev.footprint_reason(self.bev.certificate["xy"]) or "safe") if self.bev.certificate else "unavailable",startup_footprint=self.bev.footprint_reason(self.xy()) or "safe",frontier_components=getattr(self.bev,"frontier_components",0),rejections=self.bev.rejections,nearfield=self.bev.nearfield_stats,inferred_m2=round(float(self.bev.inferred.sum())*.01,2),blind_zone=self.bev.blind_zone,failed=self.nav_failures,free_m2=round(total*.01,2),occupied_m2=round(int(np.count_nonzero(occ==100))*.01,2),observed_m2=round(float(self.bev.seen.sum())*.01,2),scanned_m2=round(scanned*.01,2),coverage_fraction=scanned/max(total,1),targets=self.tracks.items,candidates=len(self.candidates),blocked=self.blocked)
        self.status_pub.publish(String(data=json.dumps(status)))
    def close(self):
        self.enabled=False;self.cancel()
        deadline=time.monotonic()+2
        while rclpy.ok() and time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.1)
        self.pool.shutdown(wait=False,cancel_futures=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['heuristic','vlm'],default='heuristic');p.add_argument('--observe',action='store_true');a,ros=p.parse_known_args()
    rclpy.init(args=ros);node=Demo(a.mode,a.observe)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
