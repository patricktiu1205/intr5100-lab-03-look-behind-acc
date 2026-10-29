"""Synthetic sensitivity study, not reconstruction of experiments or calibration."""
from pathlib import Path
from types import SimpleNamespace as NS
import sys,json,math,statistics
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'Shared Environment'),str(Path(__file__).resolve().parent)]
from student_controller3 import LookBehindController
from idm_controller import IDMController,IDMParams
from test_controller import Route,Section

def clip(x,a,b):return max(a,min(b,x))
def run(case,weight):
    gamma=case.get('gamma',2.8);beta=case.get('beta',2.8);lag=case.get('lag',.5)
    delay=case.get('delay',0);initial_front=case.get('front_gap',501.6);initial_rear=case.get('rear_gap',434.4)
    c=LookBehindController(Route(),[Section(0,100000)],rear_weight=weight)
    follower=IDMController(IDMParams());follower.last_cmd_v=40
    x=[5000+250+initial_front,5000,5000-250-initial_rear];v=[40*gamma]*3;u=[40]*3
    dt=.05;period=.15;history=[];past=[];records=[];commands=[];emergencies=0;shield_front=0;shield_rear=0
    for n in range(1201):
        t=n*dt
        if case.get('stop',False):q=40 if t<10 else max(0,40-10*(t-10)) if t<20 else min(40,5*(t-20))
        else:q=40 if t<10 else 40-4*(t-10) if t<15 else 20 if t<30 else min(40,20+4*(t-30))
        v[0]=q*gamma
        past.append((t,x.copy(),v.copy()))
        if n%3==0:
            sample=past[max(0,n-round(delay/dt))];ts,xs,vs=sample
            states=[NS(x=xx,y=0.,heading=0.,velocity=vv/gamma,recv_time=1000+ts) for xx,vv in zip(xs,vs)]
            for role,state in zip(['leader','student','following'],states):
                history.append(dict(role=role,t_from_start=ts,x=state.x,y=0,heading=0,velocity=state.velocity,s=state.x))
            command=c.step(*states,history=history,now=t,wall_now=1000+t)
            front_gap=xs[0]-xs[1]-250;rear_gap=xs[1]-xs[2]-250
            shield_front+=front_gap<80;shield_rear+=rear_gap<80
            u[1]=round(0 if front_gap<80 else command)
            u[2]=round(0 if rear_gap<80 else follower.next_velocity(vs[2]/gamma,vs[1]/gamma,rear_gap,period,v_max=70))
            emergencies+=c.debug['reason']=='front_emergency'
            commands.append(u[1])
        a=[0,0,0]
        for i in [1,2]:
            a[i]=clip((beta*u[i]-v[i])/lag,-80*beta,50*beta)
            nv=max(0,v[i]+a[i]*dt);x[i]+=.5*(v[i]+nv)*dt;v[i]=nv
        x[0]+=v[0]*dt
        records.append((t,x[0]-x[1]-250,x[1]-x[2]-250,*v,a[1]))
    active=[r for r in records if r[0]>=5]
    sd=lambda i:statistics.pstdev(r[i] for r in active)
    result=dict(scenario=case['name'],rear_weight=weight,true_telemetry_scale=gamma,true_command_gain=beta,plant_lag_s=lag,telemetry_delay_s=delay,
                minimum_front_gap=min(r[1] for r in records),minimum_rear_gap=min(r[2] for r in records),maximum_front_gap=max(r[1] for r in records),maximum_rear_gap=max(r[2] for r in records),
                simulated_front_shield_calls=shield_front,simulated_rear_shield_calls=shield_rear,
                mean_leader_coordinate_speed=statistics.mean(r[3] for r in active),mean_student_coordinate_speed=statistics.mean(r[4] for r in active),mean_following_coordinate_speed=statistics.mean(r[5] for r in active),
                front_target_rmse=math.sqrt(statistics.mean((r[1]-300-1.8*r[4])**2 for r in active)),
                student_to_leader_speed_sd=sd(4)/sd(3),following_to_student_speed_sd=sd(5)/sd(4),
                simulated_student_acceleration_rms=math.sqrt(statistics.mean(r[6]**2 for r in active)),
                front_emergency_calls=emergencies,command_min=min(commands),command_max=max(commands),
                finite=all(math.isfinite(z) for r in records for z in r),negative_gap_samples=sum(r[1]<0 or r[2]<0 for r in records))
    assert result['finite']
    return result
cases=[dict(name='nominal'),dict(name='tight_rear',rear_gap=180),dict(name='large_front_gap',front_gap=800),dict(name='small_front_gap',front_gap=250),dict(name='slow_plant',lag=1.0),dict(name='fast_plant',lag=.25),dict(name='telemetry_delay',delay=.3),dict(name='leader_stop',stop=True),dict(name='command_gain_low',beta=2.1),dict(name='command_gain_high',beta=3.5),dict(name='telemetry_scale_low',gamma=2.1),dict(name='telemetry_scale_high',gamma=3.5)]
if __name__=='__main__':
    records=[run(c,w) for c in cases for w in [0,.35]]
    payload={'label':'Synthetic first-order command-response sensitivity study; not physical validation.','duration_s':60,'plant_step_s':.05,'control_period_s':.15,'integer_commands':True,'controlled_gains_unchanged_across_cases':True,'following_idm':'Unmodified supplied IDM implementation; plant is an assumed first-order response.','negative_gap_case_count':sum(r['negative_gap_samples']>0 for r in records),'cases':records}
    (ROOT/'review/synthetic_scenario_results.json').write_text(json.dumps(payload,indent=2)+'\n')
    print('Cases:',len(records),'negative-gap cases:',payload['negative_gap_case_count'])
    for r in records:
        if r['rear_weight']:print(r['scenario'],'min gaps',round(r['minimum_front_gap'],1),round(r['minimum_rear_gap'],1),'speed SD ratios',round(r['student_to_leader_speed_sd'],3),round(r['following_to_student_speed_sd'],3))
