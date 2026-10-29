"""Sensor/frame adapter must reproduce the original actor's local actions."""
from pathlib import Path
import math
import numpy as np
import pytest
import torch
from deployment.local_controller import LocalController, PeerMeasurement, rotation, sample_lidar
from afor_local.environment import LocalFormationEnv, TaskConfig
from afor_local.policy import load_actor


ROOT=Path(__file__).resolve().parents[1]
BUNDLE=ROOT/'deployment'
POLICIES=ROOT/'models'
torch.set_num_threads(1)


def test_scan_frames_and_invalid_readings():
    ranges=np.linspace(.2,2,32)
    np.testing.assert_allclose(sample_lidar(ranges,0,2*math.pi/32,0),ranges,atol=1e-7)
    np.testing.assert_allclose(sample_lidar(ranges,0,2*math.pi/32,math.pi/2),
                               np.roll(ranges,8),atol=1e-7)
    np.testing.assert_allclose(sample_lidar(np.full(360,np.inf),-math.pi,2*math.pi/360,.37),2.)
    dense=np.linspace(.1,2,360)
    scan=sample_lidar(dense,0,2*math.pi/360,.007)
    index=np.mod((np.arange(32)*2*math.pi/32-.007)/(2*math.pi/360),360)
    np.testing.assert_allclose(scan,np.minimum(dense[np.floor(index).astype(int)],
                                                dense[(np.floor(index).astype(int)+1)%360]),atol=1e-7)
    for bad in (np.full(360,np.nan),np.full(360,-1)):
        with pytest.raises(ValueError):sample_lidar(bad,0,2*math.pi/360,0)
    with pytest.raises(ValueError):sample_lidar(ranges,0,2*math.pi/32,.13)
    with pytest.raises(ValueError):sample_lidar(ranges[:16],0,2*math.pi/32,0)


@pytest.mark.parametrize('formation',('line','circle','wedge'))
@pytest.mark.parametrize('yaw',(0.,.63,math.pi/2))
def test_world_frame_role_and_peer_order_match_actor(formation,yaw):
    env=LocalFormationEnv(TaskConfig(num_envs=1,formation=formation),seed=59001)
    actor,_=load_actor(POLICIES/formation/'actor.pt')
    state=actor.initial_state((1,5))
    transform=rotation(yaw)
    controllers=[LocalController(BUNDLE,formation,i,yaw) for i in range(5)]
    for tick in range(8):
        obs=env.observe()
        with torch.inference_mode(): expected,state=actor(obs,state)
        actual=[]
        for i,controller in enumerate(controllers):
            peers=[PeerMeasurement(j,(env.pos[0,j]-env.pos[0,i]).numpy()@transform.T,
                                     (env.vel[0,j]-env.vel[0,i]).numpy()@transform.T)
                   for j in reversed(range(5)) if bool(obs['mask'][0,i,j])]
            shift=np.array([1.,-2.],dtype=np.float32)
            args=dict(final_goal_world=env.goal[0,i].numpy()@transform.T+shift,
                      position_world=env.pos[0,i].numpy()@transform.T+shift,
                      velocity_world=env.vel[0,i].numpy()@transform.T,
                      scan_mission=obs['scan'][0,i].numpy()*2,peers=peers,timestamp=tick*.05)
            command=controller.step(**args)
            acceleration=command.acceleration_world@transform
            actual.append(acceleration)
            np.testing.assert_allclose(command.velocity_target_world@transform,
                np.clip(env.vel[0,i].numpy()+.05*acceleration,-1,1),atol=2e-6)
            torch.testing.assert_close(controller.memory,state[:,i],atol=2e-6,rtol=1e-5)
        torch.testing.assert_close(torch.from_numpy(np.array(actual))[None],expected,atol=3e-6,rtol=1e-5)
        env.step(expected)
    args['timestamp']=1.
    with pytest.raises(ValueError): controllers[-1].step(**args)
    controllers[-1].reset()
    assert torch.count_nonzero(controllers[-1].memory)==0
    assert controllers[-1].last_timestamp is None


def test_reject_duplicate_peers_and_empty_set_works():
    controller=LocalController(BUNDLE,'line',2)
    arguments=dict(final_goal_world=[0,3],position_world=[0,-3],velocity_world=[0,0],scan_mission=np.full(32,2.))
    controller.step(**arguments)
    peer=PeerMeasurement(1,[-.5,0],[0,0])
    with pytest.raises(ValueError):controller.step(**arguments,peers=[peer,peer])
    with pytest.raises(ValueError):controller.step(**arguments,peers=[PeerMeasurement(2,[0,0],[0,0])])
