"""Conservative evidence-based events/segments, with one placement contract."""
from dataclasses import dataclass,asdict
from itertools import permutations
import math
import numpy as np

VERSION='airport_events/1.0'
STATES=['SEARCH_TARGET','ALIGN_BASE','APPROACH_TRAY','ADJUST_TRAY','GRASP_TRAY','TRANSPORT_TRAY','PLACE_TRAY','VERIFY_PLACEMENT','RECOVER','TASK_COMPLETE']
SKILLS=['navigate_to_tray','push_adjust_tray','grasp_lift_tray','carry_tray','place_release_tray','verify_or_recover']
PICK_PHASES=['pregrasp','approach','contact','close','load_transfer','lift','stabilize']
PLACE_PHASES=['transport','preplace','descend','support_contact','unload','open','retreat']

@dataclass(frozen=True)
class Criteria:
    position_m:float=.03
    yaw_deg:float=5.
    tilt_deg:float=5.
    gap_min_m:float=0.
    gap_max_m:float=.03
    stable_s:float=1.
    max_speed_m_s:float=.01
    penetration_m:float=.003
CRITERIA=Criteria()

def angle_error(a,b):return np.arctan2(np.sin(a-b),np.cos(a-b))

def orientation(q):
    w,x,y,z=np.moveaxis(q,-1,0)
    yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
    tilt=np.arccos(np.clip(1-2*(x*x+y*y),-1,1))
    return yaw,tilt

def placement_mask(pose,table,forces,floor,illegal,penetration,spec,fps,criteria=CRITERIA):
    n,k,_=pose.shape;mask=np.zeros(n,bool);mapping=np.full((n,k),-1,int)
    if k!=3:return mask,mapping
    r=np.array(spec['belt_frame']['rotation_world_from_belt']);origin=np.array(spec['belt_frame']['origin_world'])
    local=(pose[:,:,:3]-origin)@r
    yaw,tilt=orientation(pose[:,:,3:]);yaw_belt=yaw+np.pi/2
    goals=np.array([x['position_belt'] for x in spec['slots']]);target_yaw=np.array([x['yaw_belt'] for x in spec['slots']])
    velocity=np.r_[np.zeros((1,k)),np.linalg.norm(np.diff(pose[:,:,:3],axis=0),axis=2)*fps]
    for i in range(n):
        if illegal[i] or penetration[i]>criteria.penetration_m or np.any(floor[i]) or np.any(table[i]<=0) or np.any(forces[i]>.5):continue
        if np.any(tilt[i]>np.deg2rad(criteria.tilt_deg)) or np.any(velocity[i]>criteria.max_speed_m_s):continue
        for perm in permutations(range(k)):
            ids=np.array(perm)
            if np.any(np.linalg.norm(local[i,ids]-goals,axis=1)>criteria.position_m):continue
            period=float(spec.get('yaw_period_rad',2*np.pi))
            if period not in (np.pi,2*np.pi):raise ValueError('Supported yaw periods are pi and 2*pi')
            scale=2*np.pi/period
            if np.any(abs(angle_error(yaw_belt[i,ids]*scale,target_yaw*scale)/scale)>np.deg2rad(criteria.yaw_deg)):continue
            ext=.28*abs(np.cos(yaw_belt[i,ids]))+.215*abs(np.sin(yaw_belt[i,ids]))
            gap=np.diff(local[i,ids,0])-ext[:-1]-ext[1:]
            if np.all((gap>=criteria.gap_min_m-1e-6)&(gap<=criteria.gap_max_m+1e-6)):
                mask[i]=True;mapping[i]=ids;break
    return mask,mapping

def annotate_arrays(a,spec,episode_id,controller_source,operator_result='unlabeled',fps=20):
    pose=a['pose'];n,k,_=pose.shape;force=a['forces'];table=a['table'];support=a['support'];floor=a['floor'];state=a['state'];tcp=a['tcp'];bottom=a['bottom']
    both=np.all(force>.5,axis=2);any_contact=np.any(force>.5,axis=2)
    yaw,tilt=orientation(pose[:,:,3:]);speed=np.r_[np.zeros((1,k)),np.linalg.norm(np.diff(pose[:,:,:3],axis=0),axis=2)*fps]
    yaw_speed=np.r_[np.zeros((1,k)),abs(angle_error(yaw[1:],yaw[:-1]))*fps]
    valid,mapping=placement_mask(pose,table,force,floor,a['illegal'],a['penetration'],spec,fps)
    required=int(math.ceil(CRITERIA.stable_s*fps))+1;stable=0;complete=np.zeros(n,bool);events=[];segments=[];labels=[]
    held=np.zeros(k,bool);air=np.zeros(k,bool);ever_lift=np.zeros(k,bool);released=np.zeros(k,bool);lost=np.zeros(k,bool);baseline=bottom[0].copy()
    closed=state[:,[7,15]]<.035;open_=state[:,[7,15]]>.06
    base_available=bool(spec.get('base_control_available',False))
    base_motion=np.zeros(n,bool)
    if base_available and 'base_pose' in a:
        b=a['base_pose'];base_motion[1:]=(np.linalg.norm(np.diff(b[:,:2],axis=0),axis=1)*fps>.005)|(abs(angle_error(b[1:,2],b[:-1,2]))*fps>.01)
    active=None;ambiguous_frames=0
    def event(i,kind,t=None,**extra):
        events.append(dict(episode_id=episode_id,scene_id=spec['scene_id'],frame=i,event=kind,target_tray_id=None if t is None else f'tray_{t}',controller_source=controller_source,environment_source='simulation',annotation_source='simulation_truth',annotation_version=VERSION,**extra))
    for i in range(n):
        if base_motion[i] and (i==0 or not base_motion[i-1]):event(i,'base_motion_started')
        if i and not base_motion[i] and base_motion[i-1]:event(i,'base_motion_stopped')
        stable=stable+1 if valid[i] else 0;complete[i]=stable>=required
        if complete[i] and (i==0 or not complete[i-1]):event(i,'task_complete')
        for t in range(k):
            if any_contact[i,t] and (i==0 or not any_contact[i-1,t]):event(i,'gripper_contact',t)
            if both[i,t] and np.all(closed[i]) and not held[t]:
                event(i,'regrasp' if lost[t] else 'both_grippers_closed',t);held[t]=True;lost[t]=False
            lifted=held[t] and bottom[i,t]-baseline[t]>.02 and support[i,t]==0
            if lifted and not air[t]:event(i,'lift_above_2cm',t);air[t]=True;ever_lift[t]=True
            if air[t] and table[i,t]>0:
                event(i,'table_support_restored',t);air[t]=False
            if held[t] and not any_contact[i,t]:
                held[t]=False
                if air[t]:event(i,'contact_lost_in_air',t);lost[t]=True
            if ever_lift[t] and not released[t] and not any_contact[i,t] and support[i,t]>0 and np.all(open_[i]):
                event(i,'open_release',t);released[t]=True
            if floor[i,t]>0 and (i==0 or floor[i-1,t]==0):event(i,'dropped_to_floor',t)
            moving=table[i,t]>0 and any_contact[i,t] and (speed[i,t]>.015 or yaw_speed[i,t]>np.deg2rad(3))
            if moving and (i==0 or speed[i-1,t]<=.015):event(i,'table_push_or_rotate',t)
        if active is not None and released[active] and not any_contact[i,active] and np.min(np.linalg.norm(tcp[i]-pose[i,active,:3],axis=1))>.27:active=None
        candidates=np.flatnonzero(any_contact[i])
        if len(candidates)==1:active=int(candidates[0])
        elif len(candidates)>1:active=None
        elif active is None:
            dist=np.linalg.norm(pose[i,:,:3,None]-tcp[i].T[None,:,:],axis=1).min(axis=1)
            near=np.flatnonzero(dist<.2)
            if len(near)==1:active=int(near[0])
        label=('SEARCH_TARGET','verify_or_recover','unknown',None,True,None)
        if complete[i]:label=('TASK_COMPLETE','verify_or_recover','verified',None,False,None)
        elif np.any(floor[i]) or a['illegal'][i] or a['penetration'][i]>CRITERIA.penetration_m:
            label=('RECOVER','verify_or_recover','recover',None,True,active)
        elif active is not None:
            t=active;category='pick';needs=False
            if lost[t]:st,skill,phase,category,needs='RECOVER','verify_or_recover','recover',None,True
            elif air[t]:
                if speed[i,t]<.01:st,skill,phase='TRANSPORT_TRAY','grasp_lift_tray','stabilize'
                elif i and pose[i,t,2]-pose[i-1,t,2]<-.0003:st,skill,phase,category='PLACE_TRAY','place_release_tray','descend','place'
                else:st,skill,phase='TRANSPORT_TRAY','carry_tray','transport';category='place'
            elif ever_lift[t] and table[i,t]>0:
                st,skill,category='PLACE_TRAY','place_release_tray','place'
                phase='open' if released[t] else 'unload' if np.all(open_[i]) else 'support_contact'
                if released[t] and np.min(np.linalg.norm(tcp[i]-pose[i,t,:3],axis=1))>.25:phase='retreat'
            elif table[i,t]>0 and any_contact[i,t] and (speed[i,t]>.015 or yaw_speed[i,t]>.05):st,skill,phase,category='ADJUST_TRAY','push_adjust_tray','adjust','adjust'
            elif both[i,t] and np.all(closed[i]):st,skill,phase='GRASP_TRAY','grasp_lift_tray','load_transfer'
            elif any_contact[i,t]:st,skill,phase='GRASP_TRAY','grasp_lift_tray','close' if np.all(closed[i]) else 'contact'
            else:st,skill,phase,needs='APPROACH_TRAY','grasp_lift_tray','approach',True
            if i and any(e['event']=='lift_above_2cm' and e['frame']==i and e['target_tray_id']==f'tray_{t}' for e in events):st,skill,phase,category='GRASP_TRAY','grasp_lift_tray','lift','pick'
            label=(st,skill,phase,category,needs,t)
        if base_motion[i] and not np.any(any_contact[i]) and label[0] not in ('RECOVER','TASK_COMPLETE'):label=('ALIGN_BASE','navigate_to_tray','unknown',None,True,None)
        if label[4]:ambiguous_frames+=1
        labels.append(label)
    if operator_result=='aborted':event(n-1,'manual_abort')
    success=bool(complete[-1] and operator_result not in ('aborted','failure') and not np.any(floor) and not np.any(a['illegal']) and np.max(a['penetration'])<=CRITERIA.penetration_m)
    result='success' if success else 'aborted' if operator_result=='aborted' else 'failure' if operator_result=='failure' or np.any(floor) else 'needs_review'
    start=0
    for end in range(1,n+1):
        if end<n and labels[end]==labels[start]:continue
        st,skill,phase,category,needs,t=labels[start]
        segments.append(dict(segment_id=f'{episode_id}_{start:06d}',episode_id=episode_id,scene_id=spec['scene_id'],start_frame=start,end_frame=end,state=st,skill=skill,phase=phase,primary_class=category,target_tray_id=None if t is None else f'tray_{t}',target_slot_id=None,success=success and not needs,recovery=st=='RECOVER',failure_reason=None if success else ('base_control_unavailable' if st=='SEARCH_TARGET' and not base_available else 'not_verified'),needs_review=needs or (operator_result=='success' and not success),controller_source=controller_source,environment_source='simulation',annotation_source='simulation_truth',annotation_version=VERSION))
        start=end
    return dict(events=events,segments=segments,result=result,success=success,operator_result=operator_result,operator_success_disagrees=operator_result=='success' and not success,needs_review_frames=ambiguous_frames,criteria=asdict(CRITERIA),base_control_available=base_available)
