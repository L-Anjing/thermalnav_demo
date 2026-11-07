import unittest,copy
import numpy as np
from thermalnav.core import BEV
from thermalnav.nearfield import complete,blind_zone

class NearfieldTests(unittest.TestCase):
 def proof(self,**kw):
  p=dict(provider='thermalnav_physx_startup_v1',frame='odom',collision_free=True,ground_supported=True,initial_pose=True,xy=[0.,0.],radius=.35,stamp=1.,sim_pid=99,query_count=10);p.update(kw);return p
 def endpoints(self,x=1.5):return np.array([[x,0,0],[x,.01,0],[x,.02,0],[x,.03,0]])
 def test_inference_never_makes_free(self):
  b=BEV();before=b.occupancy().copy();s=complete(b,self.endpoints(),np.zeros(2))
  self.assertGreater(s['inferred_added'],5);np.testing.assert_array_equal(before,b.occupancy());self.assertFalse(b.scanned.any())
  self.assertEqual(b.footprint_reason((0,0)),'unknown_footprint')
 def test_completion_stops_at_wall_and_leg(self):
  for y in (0.,.02):
   b=BEV();x,yc=b.cell((.7,y));b.seen[yc,x]=True;b.log[yc,x]=2
   complete(b,self.endpoints(),np.zeros(2));x,yc=b.cell((1.,0));self.assertFalse(b.inferred[yc,x])
 def test_diagonal_ray_does_not_cut_obstacle_corner(self):
  b=BEV();x,y=b.cell((.35,.25));b.seen[y,x]=True;b.log[y,x]=3
  points=np.array([[1.,1.,0],[1.01,1.01,0],[1.02,1.02,0]])
  complete(b,points,np.zeros(2));x,y=b.cell((.65,.65));self.assertFalse(b.inferred[y,x])
 def test_missing_far_noisy_endpoints_no_completion(self):
  for points in (np.zeros((0,3)),[[np.nan,0,0]],self.endpoints(2.1),[[1.,0,0]],[[1.,0,0],[1.6,0,0],[1.9,0,0]]):
   b=BEV();complete(b,points,np.zeros(2));self.assertFalse(b.inferred.any())
 def test_actual_camera_blind_zone(self):
  K=np.array([[733,0,320],[0,733,180],[0,0,1.]])
  R=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
  a=blind_zone(K,R,np.array([.32487,0,.4]),640,360,np.zeros(2))
  self.assertAlmostEqual(a['floor_blind_left_center_right_m'][1],.32487+.4*733/179,places=5)
  self.assertNotAlmostEqual(a['floor_blind_left_center_right_m'][1],1.42,places=1)
 def test_certificate_is_bounded_not_sensor_free(self):
  b=BEV();self.assertTrue(b.accept_certificate(self.proof(),np.zeros(2),1.1))
  self.assertIsNone(b.footprint_reason((0,0)));self.assertFalse(b.seen.any());self.assertTrue(np.all(b.occupancy()==-1))
  self.assertEqual(b.footprint_reason((.1,0)),'unknown_footprint');self.assertIsNotNone(b.path_reason([(0,0),(.1,0)]))
 def test_certificate_cannot_cover_obstacle(self):
  b=BEV();x,y=b.cell((.1,0));b.seen[y,x]=True;b.log[y,x]=3
  self.assertFalse(b.accept_certificate(self.proof(),np.zeros(2),1.1));self.assertEqual(b.footprint_reason((0,0)),'occupied')
 def test_invalid_certificate_rejected(self):
  for kw in ({'collision_free':False},{'ground_supported':False},{'initial_pose':False},{'radius':2.},{'stamp':-9},{'xy':[1,0]},{'provider':'assume_safe'},{'query_count':0}):
   b=BEV();self.assertFalse(b.accept_certificate(self.proof(**kw),np.zeros(2),1.1));self.assertIsNone(b.certificate)
 def test_no_above_ground_ray_false_free(self):
  b=BEV();K=np.array([[40,0,20],[0,40,15],[0,0,1.]])
  R=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
  # This observation contains no floor endpoint in the selected height band.
  b.integrate(np.full((30,40),2.,np.float32),K,R,np.array([0,0,4.]),robot_xy=np.zeros(2))
  x,y=b.cell((.5,0));self.assertEqual(b.occupancy()[y,x],-1)
 def test_connected_safe_map_allows_route_without_manual_bootstrap(self):
  b=BEV();b.seen[130:180,130:190]=True;b.log[130:180,130:190]=-3
  self.assertTrue(b.candidates((0,0)))
 def test_certificate_does_not_bridge_unknown_gap(self):
  b=BEV();b.seen[140:160,165:190]=True;b.log[140:160,165:190]=-3
  self.assertTrue(b.accept_certificate(self.proof(),np.zeros(2),1.1));self.assertEqual(b.candidates((0,0)),[])
  self.assertGreater(b.rejections['disconnected']+b.rejections['no_safe_path'],0)
if __name__=='__main__':unittest.main()
