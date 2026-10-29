"""Run the actual unmodified TA GUI loop in a disposable offline environment."""
from pathlib import Path
import json,os,shutil,subprocess,sys,tempfile
ROOT=Path(__file__).resolve().parents[1]
CHILD=r'''
import time,json,socket
from types import SimpleNamespace as NS
from unittest.mock import patch
from PyQt6.QtWidgets import QApplication,QMessageBox
import student_controller3 as submission
from ta_experiment3_gui import Experiment3GUI
from vehicle_io import VehicleState
from idm_controller import IDMController

def xy(route,s):
    s%=route.loop_length
    i=max(i for i,p in enumerate(route.points) if p.s<=s)
    p,q=route.points[i],route.points[(i+1)%len(route.points)]
    f=(s-p.s)/route.segment_length(i)
    return p.x+f*(q.x-p.x),p.y+f*(q.y-p.y)
class Sink:
    def __init__(self):self.commands=[]
    def set_velocity(self,v):assert isinstance(v,int) and 0<=v<=100;self.commands.append(v)
    def close(self):pass
app=QApplication([])
with patch.object(QMessageBox,'warning',return_value=None),patch.object(socket.socket,'connect',side_effect=AssertionError('Network forbidden')):
    w=Experiment3GUI(submission.student_control);w.ui_timer.stop();assert w.route_valid
    w.idm_controller=IDMController(w._idm_params());w.recording=True;w.run_start_wall=time.time()
    w.command_clients={r:Sink() for r in w.roles};states={}
    w.state_clients={r:NS(latest=lambda r=r:states[r],stop=lambda:None) for r in w.roles}
    sec=w.control_sections[-1];clock=[100.];submission._CLOCK=lambda:clock[0]
    samples=[];timings=[]
    def telemetry(offsets,seq):
        for role,s in zip(w.roles,offsets):
            x,y=xy(w.route,s)
            states[role]=VehicleState(role,'synthetic',time.time(),seq,dict(x=x,y=y,velocity=40.,heading=0.,center_distance=1.))
    with patch('time.monotonic',side_effect=lambda:clock[0]):
        for n in range(35):
            clock[0]=100+n*.2
            telemetry([sec.start_s+2000+n*4,sec.start_s+1250+n*4,sec.start_s+550+n*4],n)
            w._record_states();start=time.perf_counter();w._control_step();timings.append((time.perf_counter()-start)*1000)
            samples.append(submission.student_control.last_debug)
            assert w.recorder.command_rows[-1]['mode']=='student_straight'
            assert w.recorder.command_rows[-2]['mode']=='idm_straight'
            assert samples[-1]['reason']=='feedback',samples[-1]
        # Rear on another section: student active, following IDM bypasses.
        clock[0]+=1;telemetry([sec.start_s+2000,sec.start_s+1250,w.control_sections[0].start_s+300],36)
        w._record_states();w._control_step()
        assert w.recorder.command_rows[-2]['mode']=='leader_follow_noncontrol'
        assert w.recorder.command_rows[-1]['mode']=='student_straight'
        assert not submission.student_control.last_debug['rear_valid']
        # Leader leaves common section: callback is bypassed by TA.
        clock[0]+=1;telemetry([w.control_sections[0].start_s+300,sec.start_s+1250,sec.start_s+550],37)
        w._record_states();w._control_step();assert w.recorder.command_rows[-1]['mode']=='leader_follow_noncontrol'
        # Front spacing emergency must stop the actual sent student command.
        clock[0]+=1;telemetry([sec.start_s+1500,sec.start_s+1250,sec.start_s+550],38)
        w._record_states();w._control_step();assert w.command_clients['student'].commands[-1]==0
    assert not any(row['mode']=='student_fallback' for row in w.recorder.command_rows)
    result={'active_cycles':35,'command_rows':len(w.recorder.command_rows),'rear_bypass_checked':True,'front_bypass_checked':True,'spacing_stop_checked':True,'callback_exception_fallbacks':0,'maximum_complete_gui_cycle_ms':max(timings),'synthetic_only':True,'network_forbidden':True}
    w.recording=False;w.close();print(json.dumps(result))
'''
with tempfile.TemporaryDirectory(prefix='intr5100-lab3-gui-') as tmp:
    tmp=Path(tmp)
    for p in (ROOT/'environment').iterdir():
        if p.is_file() and p.suffix in {'.py','.yaml'}:shutil.copyfile(p,tmp/p.name)
    shutil.copyfile(ROOT/'student_controller3.py',tmp/'student_controller3.py')
    (tmp/'check.py').write_text(CHILD)
    env=dict(os.environ,QT_QPA_PLATFORM='offscreen',PYTHONDONTWRITEBYTECODE='1',MPLCONFIGDIR=str(tmp/'cache'))
    result=subprocess.run([sys.executable,str(tmp/'check.py')],cwd=tmp,env=env,capture_output=True,text=True,check=True)
    record=json.loads(result.stdout.strip().splitlines()[-1])
    (ROOT/'review/ta_integration_checks.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2))
