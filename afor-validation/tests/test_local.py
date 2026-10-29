"""Locality, physical transitions, recurrent PPO, and reproducible deployment."""
from pathlib import Path
import json
import numpy as np
import pytest
import torch
from afor_local.environment import LocalFormationEnv, TaskConfig
from afor_local.observation import SensorConfig, pack_observation, lidar_scan
from afor_local.policy import ActorCritic, LocalActor, PolicyConfig, load_actor, export_actor
from afor_local.train import PPOConfig, advantages, collect, sequence_view, update, train
from afor_local.evaluate import independent_clearances, evaluate
from afor_local.barrier import local_braking_loss

torch.set_num_threads(1)


@pytest.mark.parametrize('scenario',LocalFormationEnv.names)
def test_randomized_maps_have_valid_endpoints_and_local_contract(scenario):
    env = LocalFormationEnv(TaskConfig(num_envs=128,scenario=scenario),seed=81)
    assert (env.clearances(env.pos,env.pos)>0).all()
    assert (env.clearances(env.goal,env.goal)>0).all()
    assert set(env.observe()) == {'ego','scan','neighbours','mask'}
    assert not any(hasattr(env,k) for k in ('stage','waypoint','entry','exit','desired_lengths'))
    old = env.vertices.clone()
    env.reset()
    assert torch.equal(old,env.vertices) == (scenario == 'narrowing')


@pytest.mark.parametrize('formation',['line','circle','wedge'])
def test_narrowing_preserves_original_geometry_endpoints_and_success(formation):
    fixture = json.loads((Path(__file__).parent/'fixtures/original_scene.json').read_text())
    expected = fixture['formations'][formation]
    env = LocalFormationEnv(TaskConfig(num_envs=4,formation=formation),seed=93)
    for ids, frame in zip((None,torch.tensor([0,3])), expected['resets']):
        if ids is not None:
            env.reset(ids)
        torch.testing.assert_close(env.pos,torch.tensor(frame['position']),rtol=0,atol=0)
        torch.testing.assert_close(env.goal,torch.tensor(frame['goal']),rtol=0,atol=0)
        torch.testing.assert_close(env.vertices[:,:2],torch.tensor(expected['walls']).expand(4,-1,-1,-1))
        torch.testing.assert_close(env.bounds,torch.tensor(expected['physical_bounds']))
        assert env.obstacle_mask.sum(-1).eq(2).all()
        assert not any(hasattr(env,k) for k in ('stage','waypoint','entry','exit','passage_entry','desired_lengths'))
    env.pos = torch.tensor(expected['perturbed_position'])
    assert env.final_formation_ok().tolist() == expected['final_formation_ok']
    env.pos = env.goal.clone(); env.hold.fill_(9)
    _,term,_,info = env.step(torch.zeros_like(env.vel))
    assert term.all() and info['success'].all()


def test_lidar_analytic_ranges_and_peer_occlusion():
    sensors = SensorConfig(lidar_rays=8,lidar_range=4)
    pos = torch.tensor([[[0.,0.],[1.,0.]]])
    box = torch.tensor([[[[2.,-1.],[2.1,-1.],[2.1,1.],[2.,1.]]]])
    mask = torch.ones(1,1,dtype=torch.bool)
    scan = lidar_scan(pos,.1,box,mask,torch.tensor([3.,3.]),sensors)
    assert scan[0,0,0].item() == pytest.approx(.9,abs=1e-6)
    assert scan[0,1,0].item() == pytest.approx(1.,abs=1e-6)
    assert scan[0,0,2].item() == pytest.approx(3.,abs=1e-6)
    assert scan[0,1,4].item() == pytest.approx(.9,abs=1e-6)
    assert torch.isfinite(scan).all()


def test_packing_discards_invisible_and_nan_padding():
    sensors = SensorConfig(lidar_rays=8)
    peer = torch.zeros(3,8)
    peer[0,0] = .5
    peer[1] = float('nan')
    peer[2,0] = 2.1
    mask = torch.tensor([True,False,True])
    obs = pack_observation(torch.tensor([8.,0.]),torch.zeros(2),.13,torch.ones(8),peer,mask,sensors)
    assert obs['ego'].tolist() == pytest.approx([1,0,0,0,.13])
    assert obs['mask'].tolist() == [True,False,False]
    assert obs['neighbours'][1:].count_nonzero() == 0
    peer[2,2:] = 99
    other = pack_observation(torch.tensor([8.,0.]),torch.zeros(2),.13,torch.ones(8),peer,mask,sensors)
    assert all(torch.equal(obs[k],other[k]) for k in obs)


def test_remote_state_and_map_do_not_leak_and_translation_is_invariant():
    env = LocalFormationEnv(TaskConfig(num_envs=1))
    # Synthetic open sensor fixture, not an additional task/scenario.
    env.bounds.fill_(5.5); env.obstacle_mask.zero_()
    env.pos[0] = torch.tensor([[-.5,0.],[0.,0.],[.5,0.],[3.5,0.],[4.2,0.]])
    env.goal[:] = env.pos+torch.tensor([0.,1.])
    before = env.observe()
    env.pos[0,3:] += torch.tensor([0.,1.])
    env.goal[0,3:] += 20
    env.vertices[:] = torch.tensor([[3.,3.],[4.,3.],[4.,4.],[3.,4.]])
    env.obstacle_mask[:] = True
    after = env.observe()
    for key in before:
        assert torch.equal(before[key][0,0],after[key][0,0])
    env.obstacle_mask[:] = False
    env.pos += torch.tensor([.1,.1]); env.goal += torch.tensor([.1,.1])
    translated = env.observe()
    for key in after:
        torch.testing.assert_close(after[key][0,0],translated[key][0,0],atol=1e-6,rtol=1e-6)


def test_actor_neighbour_permutation_and_robot_relabeling():
    env = LocalFormationEnv(TaskConfig(num_envs=2))
    actor = LocalActor()
    obs = env.observe()
    state = actor.initial_state((2,5))
    action,_ = actor(obs,state)
    permutation = torch.tensor([3,1,4,0,2])
    shuffled = {**obs,'neighbours':obs['neighbours'][...,permutation,:], 'mask':obs['mask'][...,permutation]}
    torch.testing.assert_close(action,actor(shuffled,state)[0])
    shuffled = {k:v[:,permutation] for k,v in shuffled.items()}
    torch.testing.assert_close(action[:,permutation],actor(shuffled,state)[0])


def test_masked_peer_nan_does_not_change_actor():
    env = LocalFormationEnv(TaskConfig(num_envs=2))
    obs = env.observe()
    actor = LocalActor()
    state = actor.initial_state((2,5))
    expected = actor(obs,state)[0]
    obs['neighbours'][~obs['mask']] = float('nan')
    torch.testing.assert_close(expected,actor(obs,state)[0])


@pytest.mark.parametrize('width',[1.1,1.2,1.3,1.4,1.5,1.6])
def test_sweeps_match_independent_numpy_oracle(width):
    env = LocalFormationEnv(TaskConfig(num_envs=16,passage_width=width),seed=14)
    generator = torch.Generator().manual_seed(51)
    start = torch.rand(16,5,2,generator=generator)*8-4
    end = torch.rand(16,5,2,generator=generator)*8-4
    np.testing.assert_allclose(env.clearances(start,end).numpy(),independent_clearances(env,start,end),atol=1e-9,rtol=1e-7)


def test_crossing_between_endpoints_is_terminal_no_rollback_continuation():
    env = LocalFormationEnv(TaskConfig(num_envs=1,dt=1))
    env.bounds.fill_(5.5); env.obstacle_mask.zero_()
    env.pos[0] = torch.tensor([[-.3,0.],[.3,0.],[-2.,2.],[0.,2.],[2.,2.]])
    env.vel.zero_(); env.vel[0,0,0] = 1; env.vel[0,1,0] = -1
    original = env.pos.clone()
    candidate = env.pos+env.vel
    assert (env.clearances(candidate,candidate)>0).all()
    reward,term,trunc,info = env.step(torch.zeros_like(env.vel))
    assert term.all() and not trunc.any() and info['collision'].all() and not info['success'].any()
    assert torch.equal(original,env.pos)
    assert (reward<=-10).all()
    with pytest.raises(RuntimeError,match='Reset'):
        env.step(torch.zeros_like(env.vel))


def test_goal_success_needs_no_team_phase_and_limits_hold():
    env = LocalFormationEnv(TaskConfig(num_envs=2))
    env.pos = env.goal.clone(); env.vel.zero_(); env.hold.fill_(9)
    _,term,_,info = env.step(torch.zeros_like(env.vel))
    assert term.all() and info['success'].all()
    env.reset()
    for _ in range(8):
        previous = env.vel.clone()
        env.step(torch.full_like(env.vel,5))
        assert ((env.vel-previous).abs()<=env.dt+1e-6).all()
        assert (env.vel.abs()<=1).all()


def test_time_limit_bootstrap_does_not_cross_reset():
    reward = torch.tensor([[[1.]],[[100.]]])
    value = torch.zeros_like(reward)
    next_value = torch.tensor([[[2.]],[[5.]]])
    term = torch.tensor([[False],[True]])
    trunc = torch.tensor([[True],[False]])
    adv,ret = advantages(reward,value,next_value,term,trunc,.9,.95)
    torch.testing.assert_close(adv,torch.tensor([[[2.8]],[[100.]]]))
    torch.testing.assert_close(adv,ret)


def test_recurrent_reset_replay_and_real_optimizer_update():
    torch.manual_seed(90)
    env = LocalFormationEnv(TaskConfig(num_envs=2,horizon=5))
    model = ActorCritic()
    initial = model.initial_state((2,5))
    obs = env.observe()
    reset = torch.ones(2,5,dtype=torch.bool)
    actor_initial = model.actor.initial_state((2,5))
    torch.testing.assert_close(model.actor(obs,actor_initial)[0],model.actor(obs,torch.randn_like(actor_initial),reset)[0])
    config = PPOConfig(steps=8,sequence=4,epochs=1,sequences_per_batch=5)
    data,_,_,episodes = collect(model,env,initial,reset,config)
    assert len(episodes)==2
    sequences = {k:sequence_view(v,4) for k,v in data.items() if k!='obs'}
    sequence_obs = {k:sequence_view(v,4) for k,v in data['obs'].items()}
    state = sequences['state'][:,0]
    for t in range(4):
        _,_,logprob,_,state = model({k:v[:,t] for k,v in sequence_obs.items()},state,sequences['reset'][:,t],sequences['latent'][:,t])
        torch.testing.assert_close(logprob,sequences['logprob'][:,t],atol=1e-6,rtol=1e-6)
    before = model.actor.mean.weight.detach().clone()
    metrics = update(model,torch.optim.Adam(model.parameters(),lr=3e-4),data,config)
    assert metrics['optimizer_steps']==4
    assert all(np.isfinite(v) for v in metrics.values())
    assert not torch.equal(before,model.actor.mean.weight)


def test_export_works_for_single_robot_without_environment_or_value_head(tmp_path):
    model = ActorCritic()
    export_actor(model,tmp_path/'actor.pt',.05)
    actor,dt = load_actor(tmp_path/'actor.pt')
    assert not hasattr(actor,'value') and dt==.05
    obs = pack_observation(torch.tensor([0.,5.]),torch.zeros(2),.13,torch.full((32,),2.),
                           torch.zeros(0,8),torch.zeros(0,dtype=torch.bool))
    with torch.no_grad():
        expected = model.actor(obs,model.actor.initial_state())
        actual = actor(obs,actor.initial_state())
    torch.testing.assert_close(actual,expected)
    assert actual[0].shape==(2,) and torch.isfinite(actual[0]).all()


def test_exact_cpu_resume_includes_environment_optimizer_and_memory(tmp_path):
    kwargs = dict(seed=4,task=TaskConfig(num_envs=2,horizon=7),
                  ppo=PPOConfig(steps=8,sequence=4,epochs=1,sequences_per_batch=20))
    train(tmp_path/'whole',2,**kwargs)
    train(tmp_path/'split',1,**kwargs)
    train(tmp_path/'split',2,resume=tmp_path/'split/latest.pt',**kwargs)
    whole = torch.load(tmp_path/'whole/latest.pt',weights_only=False)
    split = torch.load(tmp_path/'split/latest.pt',weights_only=False)
    def equal(a,b):
        if isinstance(a,torch.Tensor):
            assert torch.equal(a,b)
        elif isinstance(a,dict):
            assert a.keys()==b.keys()
            for k in a: equal(a[k],b[k])
        elif isinstance(a,(list,tuple)):
            assert len(a)==len(b)
            for x,y in zip(a,b): equal(x,y)
        else:
            assert a==b
    equal(whole,split)


def test_independent_critic_cannot_change_actor_and_replays_both_memories():
    model=ActorCritic(independent_value=True)
    env=LocalFormationEnv(TaskConfig(num_envs=2,horizon=5))
    initial=model.initial_state((2,5))
    assert initial.shape==(2,5,128)
    obs=env.observe()
    _,_,_,value,state=model(obs,initial)
    value.square().mean().backward()
    assert all(p.grad is None for p in model.actor.parameters())
    assert any(p.grad is not None for p in model.critic.parameters())
    actor_action,actor_state=model.actor(obs,initial[...,:64])
    torch.testing.assert_close(actor_state,state[...,:64])
    torch.testing.assert_close(actor_action,model(obs,initial,deterministic=True)[0])
    config=PPOConfig(steps=8,sequence=4,independent_value=True)
    data,*_=collect(model,env,initial,torch.ones(2,5,dtype=torch.bool),config)
    state=data['state'][0]
    for t in range(8):
        _,_,lp,_,state=model({k:v[t] for k,v in data['obs'].items()},state,data['reset'][t],data['latent'][t])
        torch.testing.assert_close(lp,data['logprob'][t])


def test_evaluation_accounts_once_for_every_episode():
    actor = LocalActor()
    report,trace = evaluate(actor,.05,'narrowing','circle',3,123,horizon=8,trajectory=True)
    assert report['successes']+report['collisions']+report['timeouts']==3
    assert len(report['results'])==3 and trace['position'].shape==(9,3,5,2)


def test_narrowing_goal_completion_does_not_imply_corridor_passage(monkeypatch):
    import afor_local.evaluate as evaluator
    # Inject a final pose that has never traversed the opening. Only the
    # trajectory validator distinguishes it from a true passage completion.
    def at_goal(*args,**kwargs):
        env = LocalFormationEnv(*args,**kwargs)
        env.pos = env.goal.clone(); env.hold.fill_(9)
        return env
    monkeypatch.setattr(evaluator,'LocalFormationEnv',at_goal)
    actor = LocalActor()
    with torch.no_grad():
        actor.mean.weight.zero_(); actor.mean.bias.zero_()
    report,_ = evaluator.evaluate(actor,.05,'narrowing','line',2,123,horizon=2)
    assert report['goal_successes']==2 and report['bypass_completions']==2
    assert report['corridor_passes']==0 and report['successes']==0


def test_local_barrier_brakes_approach_but_not_retreat():
    sensors=SensorConfig(lidar_rays=8)
    scan=torch.full((8,),2.);scan[0]=.45
    obs=pack_observation(torch.tensor([0.,2.]),torch.tensor([.8,0.]),.13,
                         scan,torch.zeros(0,8),torch.zeros(0,dtype=torch.bool),sensors)
    action=torch.zeros(2,requires_grad=True)
    loss=local_braking_loss(obs,action,sensors,.05)
    loss.backward()
    assert loss>0 and action.grad[0]>0 and torch.isfinite(action.grad).all()
    obs['ego'][2]=-.8
    assert local_braking_loss(obs,action.detach(),sensors,.05)==0


def test_local_barrier_removes_moving_peer_from_static_scan():
    sensors=SensorConfig(lidar_rays=8)
    # Both robots move right at the same velocity, keeping 0.6 m separation.
    peer=torch.tensor([[.6,0.,0.,0.,.6,0.,1.,.13]])
    scan=torch.full((8,),2.);scan[0]=.47
    obs=pack_observation(torch.tensor([2.,0.]),torch.tensor([.8,0.]),.13,
                         scan,peer,torch.ones(1,dtype=torch.bool),sensors)
    action=torch.zeros(2,requires_grad=True)
    assert local_braking_loss(obs,action,sensors,.05)==0
    obs['neighbours'][0,2]=-.8
    loss=local_braking_loss(obs,action,sensors,.05)
    loss.backward()
    assert loss>0 and action.grad[0]>0 and torch.isfinite(action.grad).all()


def test_local_barrier_training_update_has_finite_gradients():
    env=LocalFormationEnv(TaskConfig(num_envs=2,horizon=8))
    model=ActorCritic(independent_value=True)
    config=PPOConfig(steps=8,sequence=4,epochs=1,sequences_per_batch=10,
                     independent_value=True,barrier_weight=1.)
    data,*_=collect(model,env,model.initial_state((2,5)),torch.ones(2,5,dtype=torch.bool),config)
    metrics=update(model,torch.optim.Adam(model.parameters(),lr=3e-4),data,config)
    assert all(np.isfinite(v) for v in metrics.values())


def test_goal_precision_cost_prefers_closer_stationary_slots_without_changing_success():
    env=LocalFormationEnv(TaskConfig(num_envs=2,goal_distance_weight=1.))
    env.pos=env.goal.clone()
    env.pos[0,:,0]+=.35
    env.pos[1,:,0]+=.05
    env.vel.zero_()
    reward,term,trunc,info=env.step(torch.zeros_like(env.vel))
    assert reward[1].mean()>reward[0].mean()
    assert not term.any() and not info['success'].any()
    assert env.hold.tolist()==[0,1]
