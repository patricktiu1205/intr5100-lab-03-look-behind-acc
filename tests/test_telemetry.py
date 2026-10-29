"""Packet-age, ordering, prediction and two-sided constraint regression checks."""
import unittest,math
from test_controller import make,scene

def fresh(t,**kw):
    states=scene(**kw)
    for st in states:
        st.x+=112*t
        st.recv_time=100+t
    return states

def step(c,t,states):return c.step(*states,now=t,wall_now=100+t)
class TelemetryTests(unittest.TestCase):
    def test_short_missing_leader_is_predicted(self):
        c=make();step(c,0,fresh(0))
        _,own,rear=fresh(.3);value=step(c,.3,(None,own,rear))
        self.assertAlmostEqual(value,40)
        self.assertEqual(c.debug['telemetry']['leader']['source'],'cached_missing')
        self.assertAlmostEqual(c.debug['gap'],501.6)
    def test_delayed_packets_align_to_common_receiver_time(self):
        c=make();states=scene()
        for st in states:st.recv_time=99.7
        value=c.step(*states,now=0,wall_now=100)
        self.assertAlmostEqual(value,40)
        self.assertAlmostEqual(c.debug['gap'],501.6)
        self.assertAlmostEqual(c.debug['telemetry']['leader']['age'],.3)
    def test_exhausted_leader_horizon_stops(self):
        c=make();step(c,0,fresh(0))
        _,own,rear=fresh(1.3)
        # Keep callback time continuous; only front packets are lost.
        self.assertEqual(c.step(None,own,rear,now=.3,wall_now=101.3),0)
    def test_cold_stale_observation_never_becomes_a_trusted_cache(self):
        c=make();front,own,rear=scene();front.recv_time=99
        for t in [0,.15,.3]:
            self.assertEqual(c.step(front,own,rear,now=t,wall_now=100),0)
            self.assertNotIn('leader',c.observations)
    def test_out_of_order_front_packet_cannot_move_vehicle_backwards(self):
        c=make();step(c,0,fresh(0));front,own,rear=fresh(.2)
        front.x=0;front.recv_time=99
        self.assertAlmostEqual(step(c,.2,(front,own,rear)),40)
        self.assertEqual(c.debug['telemetry']['leader']['source'],'out_of_order')
    def test_route_position_jump_is_rejected(self):
        c=make();step(c,0,fresh(0));front,own,rear=fresh(.2);front.x+=800
        self.assertAlmostEqual(step(c,.2,(front,own,rear)),40)
        self.assertEqual(c.debug['telemetry']['leader']['source'],'jump_rejected')
    def test_requested_command_prediction_is_causal(self):
        c=make();c.command_tape=[(100,40),(100.2,50)]
        s,v=c._predict_own(0,112,100,100.5)
        decay=math.exp(-.3/.5)
        self.assertAlmostEqual(v,140-28*decay)
        self.assertAlmostEqual(s,112*.2+140*.3-28*.5*(1-decay))
    def test_rear_command_floor_is_bounded_by_front_cap(self):
        c=make();value=step(c,0,fresh(0,front_gap=350,rear_gap=90,q2=45))
        self.assertGreaterEqual(value,c.debug['rear_floor'])
        self.assertLessEqual(value,c.debug['safe_command'])
        c=make();value=step(c,0,fresh(0,front_gap=150,rear_gap=90,q0=35,q2=45))
        self.assertFalse(c.debug['two_sided_feasible'])
        self.assertAlmostEqual(value,c.debug['safe_command'])
    def test_front_only_ablation_has_no_rear_influence(self):
        a=make(rear_weight=0);b=make(rear_weight=0)
        ca=step(a,0,fresh(0,rear_gap=90,q2=60))
        cb=step(b,0,fresh(0,rear_gap=1000,q2=10))
        self.assertAlmostEqual(ca,cb)
        self.assertEqual(a.debug['rear_floor'],0)
        self.assertEqual(a.debug['rear_term'],0)
    def test_confidence_fades_and_expires(self):
        c=make();self.assertEqual(c._confidence(.15),1)
        self.assertGreater(c._confidence(.3),c._confidence(.8))
        self.assertEqual(c._confidence(1.3),0)
    def test_old_front_data_cannot_request_further_acceleration(self):
        c=make();front,_,_=fresh(0,q0=60);step(c,0,(front,*fresh(0)[1:]))
        for t in [.2,.4,.6]:
            _,own,rear=fresh(t);step(c,t,(front,own,rear))
        before=c.last_output;_,own,rear=fresh(.8)
        self.assertLessEqual(step(c,.8,(front,own,rear)),before)
if __name__=='__main__':unittest.main()
