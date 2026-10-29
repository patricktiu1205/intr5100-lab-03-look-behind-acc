"""Matched development/held-out synthetic benchmark; no support-file mutations."""
from pathlib import Path
from types import SimpleNamespace as NS
import sys,math,json,itertools,statistics,hashlib,argparse
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'environment'),str(Path(__file__).resolve().parent)]
from student_controller3 import LookBehindController
from idm_controller import IDMController,IDMParams
from test_controller import Route,Section

def run(case,params=None,rear=True,trace=False,dt=.05,controller_class=None):
    gamma=case.get('gamma',2.8);beta=case.get('beta',2.8);lag=case.get('lag',.5);delay=case.get('delay',0.);freq=case.get('frequency_hz')
    duration=case.get('duration',80+4/freq if freq else 90);warm=case.get('warmup',80 if freq else 5)
    c=(controller_class or LookBehindController)(Route(),[Section(0,100000)],rear_weight=1 if rear else 0)
    for k,vv in (params or {}).items():setattr(c,k,vv)
    follower=IDMController(IDMParams());follower.last_cmd_v=40
    eqrear=(250+1.2*40)/math.sqrt(1-(40/70)**4)
    x=[5250+case.get('front_gap',300+1.8*40*gamma),5000.,4750-case.get('rear_gap',eqrear)]
    v=[40*gamma]*3;u=[40]*3;history=[];past=[];records=[];commands=[];fs=rs=em=0;period=case.get('control_period',.15);frozen_state=None
    stale_stops=predictions=conflicts=0
    stride=round(period/dt);assert abs(stride*dt-period)<1e-10
    def leader(t):
        if freq:return 40+case.get('amplitude',4)*math.sin(2*math.pi*freq*t)
        if case.get('stop'):return 40 if t<10 else max(0,40-10*(t-10)) if t<20 else min(40,5*(t-20))
        return 40 if t<10 else 40-4*(t-10) if t<15 else 20 if t<30 else min(40,20+4*(t-30))
    for n in range(round(duration/dt)+1):
        t=n*dt;v[0]=gamma*leader(t);past.append((t,tuple(x),tuple(v)))
        if n%stride==0:
            states=[]
            for i,role in enumerate(['leader','student','following']):
                role_delay=case.get('role_delays',{}).get(role,delay)
                ts,xs,vs=past[max(0,n-round(role_delay/dt))]
                received=t if case.get('hidden_delay') else ts
                states.append(NS(x=xs[i],y=0.,heading=0.,velocity=vs[i]/gamma,recv_time=1000+received))
            lost_role=case.get('dropout_role','following' if case.get('rear_dropout') else None)
            start=case.get('dropout_start',15.0);end=start+case.get('dropout_duration',10.0)
            if lost_role and start<=t<end:
                index=['leader','student','following'].index(lost_role)
                if frozen_state is None:frozen_state=states[index]
                states[index]=frozen_state
            for role,st in zip(['leader','student','following'],states):history.append(dict(role=role,t_from_start=st.recv_time-1000,velocity=st.velocity))
            cmd=c.step(*states,history=history,now=t,wall_now=1000+t)
            stale_stops+=c.debug.get('reason')=='invalid_or_stale_front_state'
            predictions+=any(info.get('age',0)>.15 for info in c.debug.get('telemetry',{}).values())
            conflicts+=not c.debug.get('two_sided_feasible',True)
            gf=states[0].x-states[1].x-250;gr=states[1].x-states[2].x-250;fs+=gf<80;rs+=gr<80
            u[1]=round(0 if gf<80 else cmd)
            # GUI advances private IDM memory BEFORE its separate safety shield.
            desired=follower.next_velocity(states[2].velocity,states[1].velocity,gr,period,v_max=70)
            u[2]=round(0 if gr<80 else desired)
            em+=c.debug['reason']=='front_emergency';commands.append(u[1])
        acc=[0.]+[max(-80*beta,min(50*beta,(beta*u[i]-v[i])/lag)) for i in [1,2]]
        records.append((t,x[0]-x[1]-250,x[1]-x[2]-250,*v,acc[1],*u[1:]))
        for i in [1,2]:
            nv=max(0,v[i]+acc[i]*dt);x[i]+=.5*(v[i]+nv)*dt;v[i]=nv
        x[0]+=.5*gamma*(leader(t)+leader(t+dt))*dt
    a=np.asarray(records);z=a[a[:,0]>=warm];std=np.std(z[:,3:6],axis=0)
    r=dict(case=case['name'],rear_enabled=rear,frequency_hz=freq,minimum_front_gap=float(min(a[:,1])),minimum_rear_gap=float(min(a[:,2])),maximum_front_gap=float(max(a[:,1])),maximum_rear_gap=float(max(a[:,2])),front_rmse=float(np.sqrt(np.mean((z[:,1]-300-1.8*z[:,4])**2))),mean_speeds=np.mean(z[:,3:6],axis=0).tolist(),tail_leader_sd_ratio=float(std[2]/std[0]) if std[0]>1e-10 else None,student_leader_sd_ratio=float(std[1]/std[0]) if std[0]>1e-10 else None,simulated_student_acceleration_rms=float(np.sqrt(np.mean(z[:,6]**2))),front_shield_calls=int(fs),rear_shield_calls=int(rs),emergency_calls=int(em),negative_gap_samples=int(np.sum((a[:,1]<0)|(a[:,2]<0))),command_range=[min(commands),max(commands)],finite=bool(np.isfinite(a).all()),duration_s=duration,warmup_s=warm,plant_dt_s=dt)
    overlaps=np.flatnonzero((a[:,1]<0)|(a[:,2]<0))
    r.update(first_overlap_time_s=float(a[overlaps[0],0]) if len(overlaps) else None,stale_stop_calls=int(stale_stops),prediction_calls=int(predictions),two_sided_conflict_calls=int(conflicts))
    if freq:
        w=2*math.pi*freq;design=np.column_stack((np.ones(len(z)),np.sin(w*z[:,0]),np.cos(w*z[:,0])))
        fit=np.linalg.lstsq(design,z[:,3:6],rcond=None)[0];amps=np.linalg.norm(fit[1:],axis=0)
        r.update(sine_fit_tail_gain=float(amps[2]/amps[0]),sine_fit_student_gain=float(amps[1]/amps[0]),unexplained_tail_rms=float(np.sqrt(np.mean((z[:,5]-design@fit[:,2])**2))))
    if trace:r['trace']=a
    return r

DEVELOPMENT=[dict(name=f'dev_{f}_{lag}',frequency_hz=f,lag=lag) for f in [.025,.075,.15] for lag in [.5,.8]]
HELD_OUT=[dict(name=f'held_sine_{f}',frequency_hz=f) for f in [.015,.04,.10,.22]]+[dict(name='held_delayed_sine',frequency_hz=.06,delay=.3),dict(name='held_small_amplitude',frequency_hz=.05,amplitude=2),dict(name='held_pulse'),dict(name='held_stop',stop=True),dict(name='held_tight_rear',rear_gap=180),dict(name='held_small_front_gap',front_gap=250),dict(name='held_large_front_gap',front_gap=800),dict(name='held_fast_plant',lag=.25),dict(name='held_slow_plant',lag=1.),dict(name='held_gain_low',beta=2.1),dict(name='held_gain_high',beta=3.5),dict(name='held_scale_low',gamma=2.1),dict(name='held_scale_high',gamma=3.5),dict(name='held_020_period_sine',frequency_hz=.03,control_period=.2),dict(name='held_020_period_pulse',control_period=.2),dict(name='held_rear_dropout',rear_dropout=True)]

if __name__=='__main__':
    from evaluate_refinement import evaluate
    evaluate()
