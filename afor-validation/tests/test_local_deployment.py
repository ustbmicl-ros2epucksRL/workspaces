"""G export identity, standalone inference and calibrated sensor-frame parity."""
from pathlib import Path
import json
import math
import shutil
import numpy as np
import pytest
import torch

from deployment.local_controller import LocalController, PeerMeasurement, rotation, sample_lidar
from afor_local.environment import LocalFormationEnv, TaskConfig
from afor_local.config import PolicyConfig, ScenarioConfig
from afor_local.policy import ActorCritic
from afor_local.checkpoints import export_actor, load_actor
from tools.export_local_deployment import export_bundle
from tools.validate_local_deployment import validate_bundle, standalone_check

torch.set_num_threads(1)


@pytest.fixture(scope='module')
def bundle(tmp_path_factory):
    root = tmp_path_factory.mktemp('canonical-g-bundle')
    sources = {}
    for index, formation in enumerate(('line', 'circle', 'wedge')):
        path = root/f'{formation}.pt'
        export_actor(ActorCritic(seed=81+index), path, .05)
        sources[formation] = path
    destination = root/'export'
    export_bundle(sources, destination)
    return destination, sources


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



def assert_controller_parity(bundle, actor, formation, yaw, ticks=5):
    env = LocalFormationEnv(TaskConfig(num_envs=1, formation=formation), seed=59001,
                            scenario=ScenarioConfig(0., 0.))
    state = actor.initial_state((1, 5))
    transform = rotation(yaw)
    controllers = [LocalController(bundle, formation, i, yaw) for i in range(5)]
    for tick in range(ticks):
        obs = env.observe()
        with torch.inference_mode():
            expected, state = actor(obs, state)
        actual = []
        for i, controller in enumerate(controllers):
            peers = [PeerMeasurement(j, (env.pos[0, j]-env.pos[0, i]).numpy()@transform.T,
                                     (env.vel[0, j]-env.vel[0, i]).numpy()@transform.T)
                     for j in reversed(range(5)) if bool(obs['mask'][0, i, j])]
            shift = np.array([1., -2.], dtype=np.float32)
            arguments = dict(final_goal_world=env.goal[0, i].numpy()@transform.T+shift,
                             position_world=env.pos[0, i].numpy()@transform.T+shift,
                             velocity_world=env.vel[0, i].numpy()@transform.T,
                             scan_mission=obs['scan'][0, i].numpy()*2,
                             peers=peers, timestamp=tick*.05)
            command = controller.step(**arguments)
            acceleration = command.acceleration_world@transform
            actual.append(acceleration)
            np.testing.assert_allclose(command.velocity_target_world@transform,
                                       np.clip(env.vel[0, i].numpy()+.05*acceleration, -1, 1), atol=2e-6)
            torch.testing.assert_close(controller.memory, state[:, i], atol=2e-6, rtol=1e-5)
        torch.testing.assert_close(torch.from_numpy(np.array(actual))[None], expected, atol=3e-6, rtol=1e-5)
        env.step(expected)
    arguments['timestamp'] = 1.
    with pytest.raises(ValueError):
        controllers[-1].step(**arguments)
    controllers[-1].reset()
    assert torch.count_nonzero(controllers[-1].memory) == 0
    assert controllers[-1].last_timestamp is None


@pytest.mark.parametrize('formation', ('line', 'circle', 'wedge'))
@pytest.mark.parametrize('yaw', (0., .63, math.pi/2))
def test_g_world_frames_and_reversed_peer_order_match_eager_actor(bundle, formation, yaw):
    directory, sources = bundle
    actor, _ = load_actor(sources[formation])
    assert_controller_parity(directory, actor, formation, yaw)


def test_reject_duplicate_peers_and_allow_empty_local_set(bundle):
    directory, _ = bundle
    controller = LocalController(directory, 'line', 2)
    arguments = dict(final_goal_world=[0, 3], position_world=[0, -3],
                     velocity_world=[0, 0], scan_mission=np.full(32, 2.))
    assert np.isfinite(controller.step(**arguments).acceleration_world).all()
    peer = PeerMeasurement(1, [-.5, 0], [0, 0])
    with pytest.raises(ValueError, match='slot_id'):
        controller.step(**arguments, peers=[peer, peer])
    with pytest.raises(ValueError, match='slot_id'):
        controller.step(**arguments, peers=[PeerMeasurement(2, [0, 0], [0, 0])])
    packed = controller.pack(**arguments, peers=[PeerMeasurement(1, [3, 0], [0, 0])])
    assert not packed[3].any()


@pytest.mark.parametrize('corruption', ('schema', 'version', 'policy', 'script_hash'))
def test_bundle_contract_and_script_identity_are_enforced(bundle, tmp_path, corruption):
    source, _ = bundle
    directory = tmp_path/'copied-bundle'
    shutil.copytree(source, directory)
    contract_path = directory/'contract.json'
    contract = json.loads(contract_path.read_text())
    if corruption == 'schema':
        contract['schema'] = 'afor_local_v1'
    elif corruption == 'version':
        contract['version'] = 2
    elif corruption == 'policy':
        contract['exports'][0]['policy'] = {'hidden': 32, 'capacity_width': 24}
    else:
        script_path = directory/'line/actor.ts'
        script_path.write_bytes(script_path.read_bytes()+b'changed')
    contract_path.write_text(json.dumps(contract))
    with pytest.raises(ValueError):
        LocalController(directory, 'line', 0)


def test_bundle_is_standalone_and_validator_compares_outputs_not_task_success(bundle):
    directory, sources = bundle
    isolated = standalone_check(directory)
    assert isolated['project_package_imported'] is False
    assert set(isolated['velocity_commands']) == set(sources)
    result = validate_bundle(directory, sources, episodes=1, horizon=2,
                             widths=(1.2,), jitters=(0., .1), seed=72001)
    assert result['standalone']['project_package_imported'] is False
    assert len(result['models']) == 3
    for model in result['models']:
        assert model['max_action_error'] <= 1e-6
        assert model['max_memory_error'] <= 1e-6
        assert len(model['cases']) == 2
        assert all(case['successes'] == 0 for case in model['cases'])
