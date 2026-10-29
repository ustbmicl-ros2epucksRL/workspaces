"""Causal-control checks for the research runner, not extra deployment recipes."""
import sys
from pathlib import Path
import pytest
import torch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from ablation_support import (ControlledModel, VARIANTS, network_observation,
                              evaluate_control, export_control, load_control)
from afor_local.policy import ActorCritic
from afor_local.environment import LocalFormationEnv, TaskConfig
from afor_local.train import collect, update, PPOConfig
from afor_local.evaluate import evaluate


def test_full_control_matches_production_parameters_rollout_and_update():
    torch.set_num_threads(1)
    torch.manual_seed(2); production=ActorCritic()
    torch.manual_seed(2); control=ControlledModel({})
    assert production.state_dict().keys()==control.state_dict().keys()
    for key in production.state_dict():
        assert torch.equal(production.state_dict()[key],control.state_dict()[key])
    ppo=PPOConfig(steps=8,sequence=4,epochs=1,sequences_per_batch=20)
    for model in (production,control):
        env=LocalFormationEnv(TaskConfig(num_envs=2,horizon=7),seed=1002)
        optimizer=torch.optim.Adam(model.parameters(),lr=ppo.lr,eps=1e-5)
        torch.manual_seed(191)
        data,_,_,_=collect(model,env,model.initial_state((2,5)),torch.ones(2,5,dtype=torch.bool),ppo)
        torch.manual_seed(192)
        update(model,optimizer,data,ppo)
    for key in production.state_dict():
        assert torch.equal(production.state_dict()[key],control.state_dict()[key]),key


@pytest.mark.parametrize('variant',['no_neighbours'])
def test_ablated_information_cannot_influence_network(variant):
    control=VARIANTS[variant]
    model=ControlledModel(control)
    env=LocalFormationEnv(TaskConfig(num_envs=2))
    obs=env.observe(); state=model.initial_state((2,5))
    changed={k:v.clone() for k,v in obs.items()}
    if variant=='no_neighbours':
        changed['neighbours'].uniform_(-100,100); changed['mask'].logical_not_()
    expected=model.actor(obs,state[..., :64])
    actual=model.actor(changed,state[..., :64])
    for a,b in zip(expected,actual): torch.testing.assert_close(a,b,rtol=0,atol=0)
    # Transforming a network view must not mutate teacher measurements.
    original={k:v.clone() for k,v in obs.items()}
    network_observation(obs,control)
    for key in obs: torch.testing.assert_close(obs[key],original[key])


def test_shared_value_reuses_actor_and_receives_real_value_gradients():
    model=ControlledModel(VARIANTS['shared_value'])
    env=LocalFormationEnv(TaskConfig(num_envs=2))
    state=model.initial_state((2,5))
    features,values,_=model.features(env.observe(),state)
    assert features is values
    assert list(model.critic.parameters())==[]
    model.value(values).square().mean().backward()
    assert model.actor.encoder[0].weight.grad is not None
    assert model.actor.encoder[0].weight.grad.abs().sum()>0


def test_research_evaluation_matches_production_acceptance_and_export(tmp_path):
    torch.manual_seed(2)
    model=ControlledModel({})
    before,_=evaluate(model.actor,.05,'narrowing','line',4,61001,passage_width=1.1)
    report,trace=evaluate_control(model.actor,'line',1.1,61001,4,True)
    for key in ('episodes','successes','collisions','timeouts'):
        assert before[key]==report[key]
    for a,b in zip(before['results'],report['results']):
        for key in ('success','collision','timeout','corridor_passed'):
            assert a[key]==b[key]
        assert a['length']==b['steps']
        assert a['min_clearance']==b['min_clearance_m']
    assert trace['active'].shape[1]==4
    export_control(model,tmp_path/'actor.pt',.05,'full')
    actor,dt=load_control(tmp_path/'actor.pt')
    obs=LocalFormationEnv(TaskConfig(num_envs=2)).observe()
    memory=actor.initial_state((2,5))
    for a,b in zip(model.actor(obs,memory),actor(obs,memory)):
        torch.testing.assert_close(a,b,rtol=0,atol=0)
    assert dt==.05


def test_ablation_statistics_use_training_seeds_and_registered_controls():
    from summarize_ablations import aggregate_variants
    assert set(VARIANTS)=={'full','no_braking','no_neighbours','shared_value','no_goal_cost'}
    rows=[]
    rates={'full':[1.,1.,1.], 'no_braking':[0.,.5,0.],
           'no_neighbours':[.5,.5,.5], 'shared_value':[.75,.75,.75],
           'no_goal_cost':[.25,.25,.25]}
    for variant,values in rates.items():
        for seed,rate in zip([2,3,4],values):
            for width in [1.1,1.2,1.3,1.4]:
                rows.append({'variant':variant,'formation':'line','training_seed':seed,'width_m':width,
                             'episodes':96,'successes':int(rate*96),'collisions':int((1-rate)*96),
                             'timeouts':0,'bypasses':0,'success_rate':rate})
    summaries=aggregate_variants(rows,list(VARIANTS))
    entry=next(r for r in summaries if r['variant']=='no_braking' and r['scope']=='training_width')
    assert entry['training_seeds']==3
    assert entry['episodes']==288
    assert entry['success_rate_mean']==pytest.approx(1/6)
    assert entry['success_rate_sd']==pytest.approx(3**.5/6)
    transferred=next(r for r in summaries if r['variant']=='no_neighbours' and r['scope']=='all_widths')
    assert transferred['episodes']==3*4*96
    assert transferred['success_rate_mean']==pytest.approx(.5)
