"""Bounded live simulation validation; captures one diagnostic snapshot in /tmp."""
import time,json,os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid,Odometry
from cv_bridge import CvBridge

def main():
 rclpy.init();n=Node('thermalnav_validation');data={};subs=[];q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 for key,topic,kind in [('status','/thermalnav/status',String),('map','/thermalnav/bev',OccupancyGrid),('coverage','/thermalnav/coverage',OccupancyGrid)]:subs.append(n.create_subscription(kind,topic,lambda m,k=key:data.update({k:m}),q))
 subs.append(n.create_subscription(Odometry,'/robot0/odom',lambda m:data.update(odom=m),qos_profile_sensor_data))
 subs.append(n.create_subscription(Image,'/robot0/front_cam/rgb',lambda m:data.update(rgb=m),qos_profile_sensor_data))
 start=n.create_client(Trigger,'/thermalnav/start');stop=n.create_client(Trigger,'/thermalnav/stop')
 if not start.wait_for_service(timeout_sec=10):raise RuntimeError('Demo missing')
 # Observe the initial stationary map before enabling autonomous goals.
 ready=time.monotonic()+180
 while time.monotonic()<ready:
  rclpy.spin_once(n,timeout_sec=.1)
  if 'status' in data:
   st=json.loads(data['status'].data)
   if st.get('map_frames',0)>=8 and st.get('certificate','unavailable')!='unavailable':break
 if 'status' not in data:raise RuntimeError('No initialized sensor map')
 print('STATIONARY',data['status'].data,flush=True)
 start.call_async(Trigger.Request());end=time.monotonic()+240;report=0
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1)
  if time.monotonic()>report and 'status' in data:
   s=json.loads(data['status'].data);print(json.dumps(s),flush=True);report=time.monotonic()+20
   if s.get('completed_by_kind',{}).get('frontier',0)>=3:break
  if 'status' in data and json.loads(data['status'].data).get('blocked','').startswith('no safe') and time.monotonic()>end-210:break
 f=stop.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
 if 'status' in data:print('FINAL',data['status'].data,flush=True)
 n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
