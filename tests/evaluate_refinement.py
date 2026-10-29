"""Frozen v3 vs archived v2 and front-only; no gain selection on validation."""
import json,hashlib
from compare_disturbance import ROOT,run,HELD_OUT
from select_refinement import V2
from student_controller3 import LookBehindController
# Defined before the first v3 validation run. These are not searched by select_refinement.
VALIDATION=[dict(name=f'new_sine_{f}',frequency_hz=f) for f in [.018,.032,.055,.085,.18]]+[
 dict(name='new_known_age_015',frequency_hz=.032,delay=.15),
 dict(name='new_known_age_035',frequency_hz=.055,delay=.35),
 dict(name='new_known_age_045',frequency_hz=.032,delay=.45),
 dict(name='new_asynchronous_age',frequency_hz=.055,role_delays={'leader':.2,'student':.1,'following':.35}),
 dict(name='new_hidden_transport_020',frequency_hz=.032,delay=.2,hidden_delay=True),
 dict(name='new_hidden_transport_035',frequency_hz=.032,delay=.35,hidden_delay=True),
 dict(name='new_hidden_transport_045',frequency_hz=.032,delay=.45,hidden_delay=True),
]+[dict(name=f'new_{role}_loss_{length}',dropout_role=role,dropout_duration=length,dropout_start=12) for role,length in [('leader',.45),('leader',.9),('leader',1.5),('student',.45),('student',.9),('following',.45),('following',.9),('following',1.5),('following',3.),('following',10.)]]+[
 dict(name='new_stop_rear_loss',stop=True,dropout_role='following',dropout_duration=.9,dropout_start=12),
 dict(name='new_period020',control_period=.2),dict(name='new_rear_loss_lag080',lag=.8,dropout_role='following',dropout_duration=.9,dropout_start=12)]


def evaluate():
    params=json.loads((ROOT/'review/improvement_selection.json').read_text())['selected']['parameters']
    c=LookBehindController();assert all(getattr(c,k)==v for k,v in params.items()),'Defaults not frozen'
    groups={}
    for label,cases in [('regression_previously_seen',HELD_OUT),('validation_not_used_for_selection',VALIDATION)]:
        rows=[]
        for case in cases:
            baseline=run(case,rear=False,controller_class=V2)
            prior=run(case,controller_class=V2)
            improved=run(case,params)
            ablation=run(case,params,rear=False)
            rows.append(dict(configuration=case,front_only_original=baseline,previous_v2=prior,improved_v3=improved,front_only_matched_v3=ablation))
            metric='sine_fit_tail_gain' if case.get('frequency_hz') else 'tail_leader_sd_ratio'
            print(case['name'],'RMSE',round(prior['front_rmse'],2),'->',round(improved['front_rmse'],2),'gain',round(prior[metric],3),'->',round(improved[metric],3),'negative',improved['negative_gap_samples'],flush=True)
        groups[label]=rows
    output=dict(label='Synthetic model comparison; not physical validation',parameters_frozen=params,controller_sha256=hashlib.sha256((ROOT/'student_controller3.py').read_bytes()).hexdigest(),groups=groups)
    (ROOT/'review/refinement_comparison.json').write_text(json.dumps(output,indent=2)+'\n')
    return output
if __name__=='__main__':evaluate()
