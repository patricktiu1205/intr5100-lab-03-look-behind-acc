"""Local sampled-data analysis with quantization, caps and switching inactive.

Jacobian includes the sampled controller filters and the exact held-command
first-order response; public IDM is differentiated numerically. No measured fit.
"""
from pathlib import Path
import sys,json,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'Shared Environment'),str(Path(__file__).resolve().parent)]
from student_controller3 import LookBehindController
from test_controller import Route,Section,state
from idm_controller import IDMController,IDMParams


def analyze(rear,lag=.5,epsilon=1e-4):
    gamma=beta=2.8;vstar=40*gamma;sfstar=300+1.8*vstar
    srstar=(250+1.2*40)/math.sqrt(1-(40/70)**4);T=.15
    def mapping(x,head):
        c=LookBehindController(Route(),[Section(0,100000)],rear_weight=1 if rear else 0)
        c.last_time=0.;c.last_section=0;c.last_output=40+x[4];c.filtered_rear=x[5];c.reference_speed=vstar+x[6]
        lead=state(5250+sfstar+x[0],(vstar+head)/gamma)
        own=state(5000,(vstar+x[1])/gamma)
        follow=state(4750-srstar-x[2],(vstar+x[3])/gamma)
        u1=c.step(lead,own,follow,now=T,wall_now=100)
        idm=IDMController(IDMParams());idm.last_cmd_v=40
        u2=idm.next_velocity(follow.velocity,own.velocity,srstar+x[2],T,v_max=70)
        exp=math.exp(-T/lag)
        def advance(v,u):return beta*u+(v-beta*u)*exp,beta*u*T+(v-beta*u)*lag*(1-exp)
        nv1,dx1=advance(vstar+x[1],u1);nv2,dx2=advance(vstar+x[3],u2)
        return np.array([x[0]+(vstar+head)*T-dx1,nv1-vstar,x[2]+dx1-dx2,nv2-vstar,u1-40,c.filtered_rear,c.reference_speed-vstar])
    zero=np.zeros(7);A=np.column_stack([(mapping(np.eye(7)[j]*epsilon,0)-mapping(-np.eye(7)[j]*epsilon,0))/(2*epsilon) for j in range(7)])
    B=(mapping(zero,epsilon)-mapping(zero,-epsilon))/(2*epsilon)
    poles=np.linalg.eigvals(A);radius=float(np.max(np.abs(poles)))
    frequencies=np.geomspace(.001,1/(2*T)*.999,300)
    curves=[]
    for f in frequencies:
        z=np.exp(2j*np.pi*f*T);response=np.linalg.solve(z*np.eye(7)-A,B)
        curves.append(dict(frequency_hz=float(f),tail_gain=float(abs(response[3])),student_gain=float(abs(response[1]))))
    return dict(rear_enabled=rear,assumed_plant_lag_s=lag,jacobian_epsilon=epsilon,
        state_order=['front_gap','student_speed','rear_gap','following_speed','previous_command','rear_filter','leader_reference'],
        spectral_radius=radius,locally_schur_stable=radius<1,
        poles=[[float(p.real),float(p.imag)] for p in poles],
        maximum_sampled_tail_gain=max(r['tail_gain'] for r in curves),frequency_points=curves,
        A=A.tolist(),B=B.tolist(),equilibrium_residual=float(np.max(np.abs(mapping(zero,0)))))


if __name__=='__main__':
    cases=[analyze(r,lag) for lag in [.25,.5,.8,1.] for r in [False,True]]
    # Numerical differentiation consistency is separate from validation of the model.
    check=analyze(True,.5,5e-5)
    assert np.allclose(cases[3]['A'],check['A'],atol=1e-5,rtol=1e-4)
    result={'label':'Local, unquantized, no-delay, unsaturated sampled-data model; not proof for switched physical testbed','period_s':.15,'derivative_consistency_checked':True,'cases':cases}
    (ROOT/'review/linear_response.json').write_text(json.dumps(result,indent=2)+'\n')
    for r in cases:print('rear',r['rear_enabled'],'lag',r['assumed_plant_lag_s'],'radius',round(r['spectral_radius'],5),'peak sampled tail gain',round(r['maximum_sampled_tail_gain'],3))
