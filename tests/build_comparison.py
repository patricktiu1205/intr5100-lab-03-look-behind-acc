"""Build current three-way scientific plots and numerical refinement evidence."""
import os,json,csv,tempfile
import numpy as np
from compare_disturbance import ROOT,run
from select_refinement import V2

def main():
    params=json.loads((ROOT/'review/improvement_selection.json').read_text())['selected']['parameters']
    out=ROOT/'results';out.mkdir(exist_ok=True)
    checks=[]
    for case in [dict(name='refine_sine_004',frequency_hz=.04),dict(name='refine_delayed_032',frequency_hz=.032,delay=.15),dict(name='refine_front_loss',dropout_role='leader',dropout_duration=.9,dropout_start=12)]:
        for old in [False,True]:
            kw=dict(controller_class=V2) if old else dict(params=params)
            a=run(case,dt=.05,**kw);b=run(case,dt=.025,**kw)
            metric='sine_fit_tail_gain' if case.get('frequency_hz') else 'tail_leader_sd_ratio'
            error=abs(a[metric]-b[metric])/max(abs(b[metric]),1e-6)
            assert error<.12,(case,old,error)
            checks.append(dict(case=case['name'],previous=old,metric=metric,coarse=a[metric],fine=b[metric],relative_change=error))
    (ROOT/'review/numerical_refinement.json').write_text(json.dumps(dict(cases=checks,maximum_relative_change=max(x['relative_change'] for x in checks)),indent=2)+'\n')
    cases=[dict(name='figure_sine',frequency_hz=.04),dict(name='figure_stop',stop=True),dict(name='figure_leader_loss',dropout_role='leader',dropout_duration=.9,dropout_start=12)]
    traces=[]
    for case in cases:
        pair=[run(case,controller_class=V2,trace=True),run(case,params,trace=True)]
        traces.append(pair)
        for label,r in zip(['previous_v2','improved_v3'],pair):
            with (out/f"{case['name']}_{label}.csv").open('w',newline='') as f:
                writer=csv.writer(f);writer.writerow(['time_s','front_gap_cu','rear_gap_cu','leader_speed_cu_s','student_speed_cu_s','following_speed_cu_s','simulated_student_acceleration_cu_s2','student_command','following_command']);writer.writerows(r['trace'])
    with tempfile.TemporaryDirectory(prefix='intr5100-lab3-v3-plot-') as tmp:
        os.environ['MPLCONFIGDIR']=tmp
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
        fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
        old=json.loads((ROOT/'archive/v2/review/linear_response.json').read_text())['cases']
        new=json.loads((ROOT/'review/linear_response.json').read_text())['cases']
        color=['#b75a19','#17689d'];labels=['Previous look-behind','Improved look-behind']
        for curves,rear,c,label in [(old,False,'#777777','Original front-only'),(old,True,color[0],labels[0]),(new,True,color[1],labels[1])]:
            rows=next(x for x in curves if x['assumed_plant_lag_s']==.5 and x['rear_enabled']==rear)['frequency_points']
            axes[0,0].semilogx([r['frequency_hz'] for r in rows],[r['tail_gain'] for r in rows],color=c,label=label)
        axes[0,0].axhline(1,color='gray',ls=':',lw=1);axes[0,0].set(xlim=(.001,1),xlabel='Frequency (Hz)',ylabel='Leader-to-follower gain',title='Local model — no delay, quantization or active guards')
        rows=traces[0];mask=rows[0]['trace'][:,0]>=130
        axes[0,1].plot(rows[0]['trace'][mask,0]-130,rows[0]['trace'][mask,3],color='black',ls=':',label='Leader')
        for r,c,label in zip(rows,color,labels):axes[0,1].plot(r['trace'][mask,0]-130,r['trace'][mask,5],color=c,label=label)
        axes[0,1].set(xlabel='Time within final two periods (s)',ylabel='Follower coordinate speed (cu/s)',title='0.04 Hz nonlinear response — integer commands')
        for r,c,label in zip(traces[1],color,labels):
            z=r['trace'];mask=z[:,0]<=50
            axes[1,0].plot(z[mask,0],z[mask,1]-300-1.8*z[mask,4],color=c,label=label)
        axes[1,0].axhline(0,color='gray',ls=':',lw=1);axes[1,0].set(xlabel='Time (s)',ylabel='Front-gap target error (cu)',title='Stopping test — same target policy and leader input')
        for r,c,label in zip(traces[2],color,labels):
            z=r['trace'];mask=(z[:,0]>=10)&(z[:,0]<=17)
            axes[1,1].step(z[mask,0],z[mask,7],where='post',color=c,label=label)
        axes[1,1].axvspan(12,12.9,color='gray',alpha=.15,label='Leader packet freeze')
        axes[1,1].set(xlabel='Time (s)',ylabel='Requested speed command',title='0.9 s front packet loss — bounded prediction')
        for ax in axes.flat:ax.grid(alpha=.2)
        axes[0,0].legend(fontsize=8);axes[0,1].legend(fontsize=8);axes[1,1].legend(fontsize=8)
        fig.suptitle('Lab III refinement — synthetic comparison, not physical validation',fontsize=13)
        fig.savefig(out/'Disturbance Comparison.png',dpi=180);plt.close(fig)
    print('Maximum refinement change:',max(x['relative_change'] for x in checks))
    print('Current plot and three paired trace files saved.')
if __name__=='__main__':main()
