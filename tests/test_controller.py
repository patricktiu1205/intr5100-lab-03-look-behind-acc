"""Deterministic logic checks with no physical vehicles or network clients."""
import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from student_controller3 import LookBehindController
class Section:
    def __init__(self,lo=0,hi=10000):self.lo,self.hi=lo,hi
    def contains(self,s,length):return self.lo<=s%length<=self.hi
class Route:
    loop_length=100000.
    def project_xy(self,x,y):return NS(s=x%self.loop_length,distance=abs(y))
    def forward_gap(self,front,rear):return (front-rear)%self.loop_length

def state(x,v=40,age=0,y=0):return NS(x=x,y=y,velocity=v,heading=0.,recv_time=100-age)
def scene(front_gap=501.6,rear_gap=(250+1.2*40)/math.sqrt(1-(40/70)**4),q0=40,q1=40,q2=40):
    return state(3250+front_gap,q0),state(3000,q1),state(2750-rear_gap,q2)
def make(**kw):return LookBehindController(Route(),[Section()],**kw)
class ControllerTests(unittest.TestCase):
    def step(self,c,states=None,t=0,h=None):return c.step(*(states or scene()),history=h,now=t,wall_now=100)
    def test_equilibrium_and_units(self):
        c=make();self.assertAlmostEqual(self.step(c),40);self.assertAlmostEqual(c.debug['target'],501.6)
        self.assertIsNone(c.debug['front_ttc'])
    def test_front_spacing_feedback(self):
        self.assertGreater(self.step(make(),scene(front_gap=650)),40)
        self.assertLess(self.step(make(),scene(front_gap=350)),40)
    def test_closing_sign(self):
        c=make();self.assertEqual(self.step(c,scene(front_gap=150,q0=0)),0)
        self.assertEqual(c.debug['reason'],'front_emergency')
        c=make();self.assertGreater(self.step(c,scene(front_gap=350,q0=60)),0);self.assertIsNone(c.debug['front_ttc'])
    def test_rear_pressure_and_front_priority(self):
        c=make();base=make(rear_weight=0);states=scene(rear_gap=200,q2=40)
        self.assertGreater(self.step(c,states),self.step(base,states));self.assertLessEqual(abs(c.debug['rear_term']),c.rear_limit)
        self.assertEqual(self.step(c,scene(front_gap=100,rear_gap=20,q2=70),.2),0)
    def test_rear_speed_error_is_damped(self):
        c=make();base=make(rear_weight=0);states=scene(q2=45)
        self.assertLess(self.step(c,states),self.step(base,states))
        self.assertLess(c.debug['rear_term'],0)
    def test_rear_slowness_request_is_disabled_when_gap_is_scarce(self):
        c=make();self.step(c,scene(q2=45))
        self.step(c,scene(rear_gap=125,q2=65),.2)
        self.assertGreaterEqual(c.debug['rear_term'],0)
    def test_rear_staleness_clears_filtered_correction(self):
        c=make();self.step(c,scene(q2=45));front,own,rear=scene()
        front.recv_time=own.recv_time=101.5
        previous=c.last_output
        output=c.step(front,own,rear,now=.2,wall_now=101.5)
        self.assertGreater(output,previous);self.assertLessEqual(output,40)
        self.assertEqual(c.debug['rear_term'],0)
        front.recv_time=own.recv_time=101.7
        self.assertAlmostEqual(c.step(front,own,rear,now=.4,wall_now=101.7),40)
    def test_invalid_rear_is_ignored(self):
        for rear in [state(90000,70),state(2300,70,age=1),NS(x=math.nan,y=0,velocity=40),None]:
            c=make();front,own,_=scene();self.assertAlmostEqual(self.step(c,(front,own,rear)),40)
            self.assertFalse(c.debug['rear_valid']);self.assertEqual(c.debug['rear_term'],0)
    def test_invalid_front_stops(self):
        for bad in [state(3700,math.nan),state(3700,40,age=1),state(3700,-1),state(3700,40,y=200)]:
            c=make();_,own,rear=scene();self.assertEqual(self.step(c,(bad,own,rear)),0)
    def test_rate_limit_and_reentry(self):
        c=make();self.step(c)
        self.assertLessEqual(self.step(c,scene(front_gap=1800,q0=70),.2),41.2+1e-9)
        self.assertLess(self.step(c,scene(front_gap=400,q0=10,q1=10,q2=10),2),12)
        self.assertTrue(c.debug['reentry'])
    def test_front_cap_overrides_rate_limit(self):
        c=make();self.step(c);value=self.step(c,scene(front_gap=180,q0=35,q1=40),.2)
        self.assertLess(value,37.6);self.assertEqual(c.debug['reason'],'front_cap')
    def test_history_reset_and_immutability(self):
        c=make();h=[dict(t_from_start=10,role='leader')];saved=[dict(h[0])]
        self.step(c,h=h);self.assertEqual(h,saved)
        self.step(c,t=.2,h=h);self.assertFalse(c.debug['reentry'])
        h.clear();self.step(c,t=.4,h=h);self.assertTrue(c.debug['reentry'])
        h.append(dict(t_from_start=1));self.step(c,t=.6,h=h)
        h[0]['t_from_start']=0;self.step(c,t=.8,h=h);self.assertTrue(c.debug['reentry'])
    def test_route_seam(self):
        c=LookBehindController(Route(),[Section(0,100000)])
        self.assertAlmostEqual(self.step(c,(state(501.6),state(99750),state(99750-250-(250+1.2*40)/math.sqrt(1-(40/70)**4)))),40)
    def test_section_transition_resets_reference(self):
        c=LookBehindController(Route(),[Section(),Section(50000,60000)])
        self.step(c,scene(q2=45))
        states=scene(q0=20,q1=20,q2=20)
        for st in states:st.x+=50000
        self.step(c,states,.2)
        self.assertTrue(c.debug['reentry'])
        self.assertAlmostEqual(c.debug['reference_speed'],56)
    def test_reverse_order(self):
        c=make();front,own,rear=scene();front.x=own.x-100
        self.assertEqual(self.step(c,(front,own,rear)),0);self.assertEqual(c.debug['reason'],'ambiguous_front_order')
    def test_wrong_section_and_missing_geometry(self):
        c=make();front,own,rear=scene();front.x=20000;self.assertEqual(self.step(c,(front,own,rear)),0)
        c=make();c.route=None
        from unittest.mock import patch
        with patch('student_controller3._load_route',side_effect=FileNotFoundError()):self.assertEqual(self.step(c),0)
        self.assertEqual(c.debug['reason'],'geometry_unavailable')
if __name__=='__main__':unittest.main()
