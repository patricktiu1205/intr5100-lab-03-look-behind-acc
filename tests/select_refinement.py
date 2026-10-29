"""Joint tracking/attenuation selection; previous evaluation cases are development now."""
import importlib.util,itertools,json,math,statistics,hashlib
from pathlib import Path
import numpy as np
from compare_disturbance import ROOT,run
from check_linear_response import analyze
spec=importlib.util.spec_from_file_location('lab3_v2',ROOT/'archive/v2/student_controller3.py');old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
V2=old.LookBehindController
DEV=[dict(name='dev_track_0015',frequency_hz=.015),dict(name='dev_track_004',frequency_hz=.04),dict(name='dev_track_0075',frequency_hz=.075),dict(name='dev_delayed_004',frequency_hz=.04,delay=.3),dict(name='dev_pulse'),dict(name='dev_stop',stop=True),dict(name='dev_short_rear_loss',dropout_role='following',dropout_duration=.6,dropout_start=12),dict(name='dev_short_front_loss',dropout_role='leader',dropout_duration=.6,dropout_start=12)]

def select():
    reference=[analyze(True,lag,controller_class=V2,frequency_count=120) for lag in [.5,.8]]
    limits=[max(1.10,r['maximum_sampled_tail_gain']+.06) for r in reference]
    candidates=[];all_local=[]
    for kf,kc,kr,kv,tau,wash in itertools.product([.15,.20,.25],[.25,.5],[.25,.35,.45],[1.,1.5,2.],[3.,6.],[12.,30.,60.]):
        params=dict(k_gap=kf,k_closing=kc,k_rear_gap=kr,k_rear_velocity=kv,reference_tau=tau,rear_washout_tau=wash)
        models=[analyze(True,lag,params=params,frequency_count=120) for lag in [.5,.8]]
        peaks=[m['maximum_sampled_tail_gain'] for m in models]
        feasible=all(m['locally_schur_stable'] and peak<=bound for m,peak,bound in zip(models,peaks,limits))
        # Tracking cost at task-relevant frequencies, without changing headway.
        cost=statistics.mean(np.interp(np.log(f),np.log([r['frequency_hz'] for r in m['frequency_points']]),[r['front_error_gain'] for r in m['frequency_points']])**2 for m in models for f in [.015,.04,.075])
        item=dict(parameters=params,local_feasible=feasible,local_tracking_cost=cost,local_peaks=peaks)
        all_local.append(item)
        if feasible:candidates.append(item)
    print('Local candidates',len(candidates),'of',len(all_local),flush=True)
    shortlist=sorted(candidates,key=lambda r:r['local_tracking_cost'])[:60]
    if not shortlist:raise RuntimeError('No attenuation-preserving candidate')
    prior=[run(c,controller_class=V2) for c in DEV]
    finalists=[]
    for item in shortlist:
        rr=[run(c,item['parameters']) for c in DEV]
        # Do not select a tracking-only improvement by allowing sustained gain>1.
        feasible=all(r['finite'] and r['negative_gap_samples']==0 and r['minimum_front_gap']>=120 and r['minimum_rear_gap']>=80 and max(r['maximum_front_gap'],r['maximum_rear_gap'])<1500 for r in rr)
        feasible=feasible and all(r['sine_fit_tail_gain']<=max(1.0,b['sine_fit_tail_gain']+.025) and r['tail_leader_sd_ratio']<=max(1.0,b['tail_leader_sd_ratio']+.025) for r,b in zip(rr,prior) if r['frequency_hz'])
        score=statistics.mean(r['front_rmse']/max(b['front_rmse'],1) for r,b in zip(rr,prior))
        feasible=feasible and score<.95 and rr[1]['front_rmse']<.95*prior[1]['front_rmse']
        # Preserve the non-sinusoidal attenuation trend; modest changes tolerated.
        score+=.5*statistics.mean(max(0,r['tail_leader_sd_ratio']-b['tail_leader_sd_ratio']) for r,b in zip(rr,prior))
        record=dict(**item,feasible=feasible,score=score,development=rr);finalists.append(record)
        print('Candidate',item['parameters'],'score',round(score,3),'feasible',feasible,flush=True)
    valid=[r for r in finalists if r['feasible']]
    if not valid:raise RuntimeError('No nonlinear feasible candidate; retain previous controller')
    best=min(valid,key=lambda x:x['score'])
    out=dict(controller_sha256=hashlib.sha256((ROOT/'student_controller3.py').read_bytes()).hexdigest(),note='Prior held-out cases are explicitly reused as development. New validation cases are separate and not used in selection.',local_grid_size=len(all_local),local_limits=limits,local_candidates=all_local,development_cases=DEV,previous_controller=prior,shortlist=finalists,selected=best)
    (ROOT/'review/improvement_selection.json').write_text(json.dumps(out,indent=2)+'\n')
    print('SELECTED',best['parameters'],'score',best['score'],flush=True)
if __name__=='__main__':select()
