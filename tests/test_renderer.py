import unittest
import numpy as np
from thermalnav.core import BEV,Candidate
from thermalnav.renderer import BEVRenderer
class RendererTests(unittest.TestCase):
 def test_semantic_palette_matches_occupancy_and_keeps_unknown(self):
  b=BEV(size=10,origin=(-5,-5));r=BEVRenderer(448,22.4)
  b.seen[70,70]=True;b.log[70,70]=-1;b.seen[70,71]=True;b.log[70,71]=1
  image=r.render(b,(0,0),0.)
  x,y=r.pixel(b.world((70,70)),(0,0));self.assertTrue(np.array_equal(image[y,x],r.FREE))
  x,y=r.pixel(b.world((71,70)),(0,0));self.assertTrue(np.array_equal(image[y,x],r.OCCUPIED))
  x,y=r.pixel(b.world((69,70)),(0,0));self.assertTrue(np.array_equal(image[y,x],r.UNKNOWN))
 def test_camera_rotation_moves_heading_arrow(self):
  b=BEV(size=10,origin=(-5,-5));r=BEVRenderer(448,22.4)
  east=r.render(b,(0,0),0.);north=r.render(b,(0,0),np.pi/2)
  self.assertFalse(np.array_equal(east,north))
 def test_coverage_render_does_not_change_occupancy(self):
  b=BEV(size=10,origin=(-5,-5));b.seen[70,70]=True;b.log[70,70]=-1
  before=b.occupancy().copy();r=BEVRenderer();a=r.render(b,(0,0),0.);c=r.coverage(b,(0,0))
  self.assertEqual(a.shape,(448,448,3));self.assertEqual(c.shape,a.shape);np.testing.assert_array_equal(before,b.occupancy())
if __name__=='__main__':unittest.main()
