import unittest
import numpy as np
from thermalnav.core import BEV,Tracks,localize_box,ray
from thermalnav.policy import Policy
class CoreTests(unittest.TestCase):
 def scene(self):
  K=np.array([[40.,0,20],[0,40.,15],[0,0,1.]])
  R=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
  return K,R
 def test_translated_pose_accumulates_in_same_world_grid(self):
  b=BEV(size=10,origin=(-5,-5));K,R=self.scene();d=np.full((30,40),2.,np.float32)
  b.integrate(d,K,R,np.array([0,0,.6]),robot_xy=np.zeros(2));first=b.occupancy().copy()
  b.integrate(d,K,R,np.array([1,0,.6]),robot_xy=np.array([1.,0]))
  self.assertGreater(np.count_nonzero(b.occupancy()!=-1),np.count_nonzero(first!=-1))
  self.assertGreater(np.count_nonzero((first!=-1)&(b.occupancy()!=-1)),0)
  self.assertAlmostEqual(b.origin[0],-5.)
 def test_camera_rotation_changes_world_projection(self):
  b=BEV(size=10,origin=(-5,-5));K,R=self.scene();d=np.full((30,40),2.,np.float32)
  b.integrate(d,K,R,np.array([0,0,.6]),robot_xy=np.zeros(2));a=b.occupancy().copy()
  yaw=np.pi/2;Z=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
  b.integrate(d,K,Z@R,np.array([0,0,.6]),robot_xy=np.zeros(2))
  self.assertFalse(np.array_equal(a,b.occupancy()))
 def test_world_height_and_map_boundary_diagnostics(self):
  b=BEV(size=4,origin=(-2,-2));K,R=self.scene();d=np.full((30,40),2.,np.float32)
  b.integrate(d,K,R,np.array([0,0,.6]),robot_xy=np.zeros(2))
  s=b.diagnostics
  self.assertEqual(s['valid_depth'],d.size)
  self.assertGreater(s['ground_points'],0);self.assertGreater(s['obstacle_points'],0)
  self.assertGreater(s['free_endpoints_in_map'],0)
  self.assertEqual(s['observed_cells'],np.count_nonzero(b.occupancy()!=-1))
 def test_grid_row_column_and_origin_round_trip(self):
  b=BEV(size=4,origin=(-2,-2));c=b.cell((.25,-.35))
  self.assertEqual(c,(22,16));np.testing.assert_allclose(b.world(c),(.25,-.35))
 def test_unknown_not_free_invalid_depth(self):
  b=BEV();b.integrate(np.full((30,40),np.nan),np.eye(3),np.eye(3),np.zeros(3));self.assertTrue(np.all(b.occupancy()==-1))
 def test_ray(self):
  self.assertEqual(list(ray((0,0),(3,0))),[(0,0),(1,0),(2,0),(3,0)])
 def test_wall_occludes_coverage_and_map(self):
  b=BEV(size=10,origin=(-5,-5));K=np.array([[40,0,20],[0,40,15],[0,0,1.]])
  R=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
  d=np.full((30,40),2.,np.float32)
  b.integrate(d,K,R,np.array([0,0,.6]),stride=1)
  x,y=b.cell((3,0));self.assertEqual(b.occupancy()[y,x],-1);self.assertFalse(b.scanned[y,x])
  x,y=b.cell((2.05,0));self.assertEqual(b.occupancy()[y,x],100)
 def test_ground_rays_clear_only_to_measured_endpoints(self):
  b=BEV(size=10,origin=(-5,-5));K,R=self.scene();d=np.full((30,40),2.,np.float32)
  b.integrate(d,K,R,np.array([0,0,.6]),stride=1)
  self.assertGreater(b.diagnostics['ray_cleared_cells'],b.diagnostics['free_endpoints_in_map'])
  # The observed vertical plane lies at x=2; no cell behind it is cleared.
  x,y=b.cell((3,0));self.assertEqual(b.occupancy()[y,x],-1)
 def test_frontier_safe_and_unreachable_rejected(self):
  b=BEV(size=10,origin=(0,0));b.seen[10:60,10:60]=True;b.log[10:60,10:60]=-3
  cs=b.candidates((3,3));self.assertTrue(cs)
  _,_,blocked=b.reachable((3,3))
  for c in cs:
   x,y=b.cell(c.xy);self.assertFalse(blocked[y,x]);self.assertGreater(c.cost,.8)
  b.log[10:60,35:40]=4
  for c in b.candidates((2,3)):self.assertLess(c.xy[0],3.5)
 def test_growth_preserves_history(self):
  b=BEV(size=2,origin=(0,0));b.seen[5,5]=True;b.log[5,5]=-2;b.scanned[5,5]=True
  b.expand([[-5,-5,0],[5,5,0]]);x,y=b.cell((.55,.55));self.assertEqual(b.occupancy()[y,x],0);self.assertTrue(b.scanned[y,x])
 def test_depth_localization_dedup(self):
  p=localize_box((0,0,20,20),np.full((20,20),2.),np.array([[10,0,10],[0,10,10],[0,0,1]]),np.eye(3),np.array([1,0,0]))
  np.testing.assert_allclose(p,[1,0,2]);t=Tracks()
  for s in (0,.6,1.2):t.add(p[:2],s)
  self.assertEqual(len(t.items),1);self.assertTrue(t.items[0]['verified'])
 def test_vlm_unavailable_is_explicit(self):
  import os
  from unittest.mock import patch
  with patch.dict(os.environ,{'THERMALNAV_VLM_URL':'','THERMALNAV_VLM_MODEL':''}):
   p=Policy('vlm');self.assertIn('VLM unavailable',p.status);self.assertEqual(p.calls,0)
if __name__=='__main__':unittest.main()
