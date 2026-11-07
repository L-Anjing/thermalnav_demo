"""Explicit simulation-only PointGoal smoke test, no Twist publisher."""
import time,math,argparse,json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import String

def main():
 p=argparse.ArgumentParser();p.add_argument('--distance',type=float,default=1.5);p.add_argument('--timeout',type=float,default=150);a=p.parse_args()
 rclpy.init();n=Node('thermalnav_manual_check');data={};pub=n.create_publisher(PoseStamped,'/goal_pose',1)
 def cb(m,k):data[k]=m
 subs=[n.create_subscription(k,t,lambda m,s=s:cb(m,s),qos_profile_sensor_data) for s,t,k in [('odom','/robot0/odom',Odometry),('cmd','/robot0/cmd_vel',Twist),('status','/thermalnav/status',String)]]
 end=time.monotonic()+15
 while 'odom' not in data and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 if 'odom' not in data:raise RuntimeError('No simulation odom')
 o=data['odom'];q=o.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));start=np.array([o.pose.pose.position.x,o.pose.pose.position.y]);goal=start+a.distance*np.array([math.cos(yaw),math.sin(yaw)])
 m=PoseStamped();m.header=o.header;m.pose.position.x=float(goal[0]);m.pose.position.y=float(goal[1]);m.pose.position.z=o.pose.pose.position.z;m.pose.orientation.w=1.
 for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
 pub.publish(m);print('MANUAL PointGoal',goal.tolist(),flush=True)
 end=time.monotonic()+a.timeout;moved=False;nonzero=False;passed=False;report=0
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1);o=data['odom'];xy=np.array([o.pose.pose.position.x,o.pose.pose.position.y]);err=np.linalg.norm(xy-goal);delta=np.linalg.norm(xy-start)
  cmd=data.get('cmd');speed=0 if cmd is None else abs(cmd.linear.x)+abs(cmd.linear.y)+abs(cmd.angular.z)
  moved |=delta>.15;nonzero |=speed>.05
  if time.monotonic()>report:print('progress displacement=%.3f error=%.3f cmd=%.3f'%(delta,err,speed),flush=True);report=time.monotonic()+10
  if moved and nonzero and err<=.75 and speed<.02:passed=True;break
 # Always terminate test by sending current-pose PointGoal to the same SRU.
 m.header=o.header;m.pose=o.pose.pose;pub.publish(m)
 for _ in range(15):rclpy.spin_once(n,timeout_sec=.1)
 print(('PASS' if passed else 'FAIL'),'manual SRU motion + arrival + zero', 'displacement',float(delta),'error',float(err),flush=True)
 if 'status' in data:print(data['status'].data,flush=True)
 n.destroy_node();rclpy.shutdown();raise SystemExit(0 if passed else 1)
if __name__=='__main__':main()
