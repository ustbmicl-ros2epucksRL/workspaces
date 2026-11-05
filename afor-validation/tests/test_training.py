"""Short real PPO runs, artifact retention, and exact interrupted-run recovery."""

from dataclasses import asdict, replace
import importlib
import json
import math

import pytest
import torch

from afor_local.environment import TaskConfig
from afor_local.observation import SensorConfig
from afor_local.train import PPOConfig
from afor_local.artifacts import file_hash, tensor_state_hash
from afor_local.checkpoints import SCHEMA, load_actor
from afor_local.config import PolicyConfig, ScenarioConfig
from afor_local.environment import LocalFormationEnv
from afor_local.policy import ActorCritic
from afor_local.protocols import TEST_SEEDS, case_specs


training = importlib.import_module('afor_local.train')
torch.set_num_threads(1)


@pytest.fixture(autouse=True)
def stable_source_identity(monkeypatch):
    # Concurrent developers may edit source during these short runs. Freeze this
    # test's actual initial identity; a separate test exercises changed sources.
    identity = training.source_hash()
    monkeypatch.setattr(training, 'source_hash', lambda: identity)


def small_recipe():
    return {
        'seed': 91,
        'task': TaskConfig(num_envs=4, horizon=11),
        'policy': PolicyConfig(),
        'ppo': PPOConfig(steps=16, sequence=8, sequences_per_batch=8, epochs=1),
        'scenario': ScenarioConfig(),
        'evaluate_final': False,
    }


def read_metrics(directory):
    return [json.loads(line) for line in (directory/'metrics.jsonl').read_text().splitlines()]


def file_snapshot(directory):
    return {path.relative_to(directory).as_posix(): path.read_bytes()
            for path in directory.rglob('*') if path.is_file()}


def assert_exact_tree(left, right):
    if isinstance(left, torch.Tensor):
        assert isinstance(right, torch.Tensor)
        assert left.dtype == right.dtype
        assert left.shape == right.shape
        assert torch.equal(left, right)
    elif isinstance(left, dict):
        assert left.keys() == right.keys()
        for key in left:
            assert_exact_tree(left[key], right[key])
    elif isinstance(left, (tuple, list)):
        assert type(left) is type(right)
        assert len(left) == len(right)
        for first, second in zip(left, right):
            assert_exact_tree(first, second)
    else:
        assert left == right


@pytest.mark.parametrize('formation', ['line', 'circle', 'wedge'])
def test_real_collect_and_ppo_update_each_formation(formation):
    torch.manual_seed(43)
    task = TaskConfig(num_envs=4, horizon=100, formation=formation)
    sensors = SensorConfig()
    ppo = PPOConfig(steps=16, sequence=8, sequences_per_batch=8, epochs=1)
    env = LocalFormationEnv(task, sensors, seed=44,
                           scenario=ScenarioConfig(perturb_probability=0., max_jitter=0.))
    model = ActorCritic(PolicyConfig(), sensors, seed=45)
    optimizer = torch.optim.Adam(model.parameters(), lr=ppo.lr, eps=1e-5)
    starts = torch.ones(4, 5, dtype=torch.bool)
    data, state, starts, episodes = training.collect(
        model, env, model.initial_state((4, 5)), starts, ppo)
    assert state.shape == (4, 5, 128)
    assert not data['latent'].requires_grad
    for key in ('advantage', 'returns', 'logprob'):
        assert torch.isfinite(data[key]).all()
    before = {key: value.clone() for key, value in model.state_dict().items()}
    metrics = training.update(model, optimizer, data, ppo, task.dt)
    assert metrics['optimizer_steps'] == 5
    assert all(math.isfinite(value) for value in metrics.values())
    for component in ('actor.', 'critic.', 'value.'):
        assert any(not torch.equal(value, before[key])
                   for key, value in model.state_dict().items() if key.startswith(component))


def test_two_updates_keep_only_actor_metrics_manifest_by_default(tmp_path):
    directory = tmp_path/'minimal-run'
    recipe = small_recipe()
    model = training.train(directory, updates=2, **recipe)
    assert {path.name for path in directory.iterdir()} == {'actor.pt', 'metrics.jsonl', 'run.json'}
    assert not (directory/'.work').exists()
    manifest = json.loads((directory/'run.json').read_text())
    assert manifest['schema'] == SCHEMA
    assert manifest['status'] == 'completed'
    assert manifest['completed_updates'] == manifest['planned_updates'] == 2
    assert manifest['config']['task'] == asdict(recipe['task'])
    assert manifest['config']['ppo'] == asdict(recipe['ppo'])
    assert manifest['config']['seed'] == recipe['seed']
    assert manifest['actor_sha256'] == file_hash(directory/'actor.pt')
    assert manifest['source_hash'] == training.source_hash()
    initial_model = ActorCritic(recipe['policy'], seed=recipe['seed'])
    assert manifest['initialization']['actor_sha256'] == tensor_state_hash(initial_model.actor.state_dict())
    initial_critic = {**{'critic.'+key: value for key, value in initial_model.critic.state_dict().items()},
                      **{'value.'+key: value for key, value in initial_model.value.state_dict().items()}}
    assert manifest['initialization']['critic_sha256'] == tensor_state_hash(initial_critic)
    assert manifest['runtime']['python']
    assert manifest['runtime']['numpy']
    assert manifest['runtime']['torch'] == str(torch.__version__)
    assert manifest['runtime']['torch_threads'] == torch.get_num_threads()
    metrics = read_metrics(directory)
    assert [row['update'] for row in metrics] == [1, 2]
    assert metrics[-1]['team_transitions'] == 2*4*16
    assert metrics[-1]['agent_transitions'] == 2*4*16*5
    assert all(row['optimizer_steps'] == 5 for row in metrics)
    for row in metrics:
        assert math.isfinite(row['policy_loss'])
        assert math.isfinite(row['value_loss'])
        assert math.isfinite(row['attention_entropy'])
    deployed, dt = load_actor(directory/'actor.pt')
    assert dt == recipe['task'].dt
    assert_exact_tree(deployed.state_dict(), model.actor.state_dict())


def test_exact_resume_matches_uninterrupted_model_optimizer_environment_and_rng(tmp_path):
    recipe = small_recipe()
    split, direct = tmp_path/'split-run', tmp_path/'direct-run'
    training.train(split, updates=1, keep_resume=True, **recipe)
    assert (split/'.work/latest.pt').is_file()
    resumed_model = training.train(split, updates=2, resume=True, keep_resume=True, **recipe)
    direct_model = training.train(direct, updates=2, keep_resume=True, **recipe)
    assert_exact_tree(resumed_model.state_dict(), direct_model.state_dict())
    resumed_checkpoint = torch.load(split/'.work/latest.pt', weights_only=False)
    direct_checkpoint = torch.load(direct/'.work/latest.pt', weights_only=False)
    for key in ('schema', 'config', 'source_hash', 'device', 'update', 'model', 'optimizer',
                'env', 'state', 'starts', 'torch_rng', 'cuda_rng'):
        assert_exact_tree(resumed_checkpoint[key], direct_checkpoint[key])
    assert resumed_checkpoint['update'] == 2
    # Time limits force real resets, exercising both saved environment RNGs.
    assert resumed_checkpoint['env']['perturbation_counters']['resets'] > recipe['task'].num_envs
    assert resumed_checkpoint['env']['perturbation_counters']['perturbed_resets'] > 0
    split_metrics, direct_metrics = read_metrics(split), read_metrics(direct)
    assert [row['update'] for row in split_metrics] == [1, 2]
    for split_row, direct_row in zip(split_metrics, direct_metrics):
        for key in split_row:
            if not key.endswith('_seconds'):
                assert split_row[key] == direct_row[key], key
    assert json.loads((split/'run.json').read_text())['completed_updates'] == 2


@pytest.mark.parametrize('keep_resume', [False, True])
def test_finalization_resume_retries_failed_evaluation_without_training(tmp_path, monkeypatch, keep_resume):
    recipe = {**small_recipe(), 'evaluate_final': True, 'eval_episodes': 1}
    directory = tmp_path/'evaluation-retry'
    evaluation = importlib.import_module('afor_local.evaluate')

    def failed_evaluation(*args, **kwargs):
        raise RuntimeError('simulated final evaluation failure')

    monkeypatch.setattr(evaluation, 'evaluate_case', failed_evaluation)
    with pytest.raises(RuntimeError, match='simulated final evaluation failure'):
        training.train(directory, updates=2, **recipe)
    manifest = json.loads((directory/'run.json').read_text())
    assert manifest['status'] == 'failed'
    assert manifest['completed_updates'] == manifest['planned_updates'] == 2
    assert not (directory/'evaluation.json').exists()
    checkpoint_path = directory/'.work/latest.pt'
    checkpoint = torch.load(checkpoint_path, weights_only=False)
    assert checkpoint['update'] == 2
    failed_files = file_snapshot(directory)
    with pytest.raises(ValueError, match='cannot precede checkpoint update'):
        training.train(directory, updates=1, resume=True, **recipe)
    assert file_snapshot(directory) == failed_files
    metrics_before = (directory/'metrics.jsonl').read_bytes()
    expected_actor, _ = load_actor(directory/'actor.pt')
    # Finalization must be able to rebuild the public actor from the retained
    # recovery checkpoint, even if its earlier export disappeared.
    (directory/'actor.pt').unlink()

    evaluation_calls = []

    def successful_evaluation(actor, dt, formation, **kwargs):
        evaluation_calls.append(kwargs)
        assert not actor.training
        assert dt == recipe['task'].dt
        assert formation == recipe['task'].formation
        assert kwargs['episodes'] == 1
        return {'seed': kwargs['seed'], 'jitter': kwargs['jitter'],
                'episodes': kwargs['episodes'], 'success_rate': 1.}, None

    def forbidden_training(*args, **kwargs):
        raise AssertionError('Finalization-only resume must not collect or optimize')

    monkeypatch.setattr(evaluation, 'evaluate_case', successful_evaluation)
    monkeypatch.setattr(training, 'collect', forbidden_training)
    monkeypatch.setattr(training, 'update', forbidden_training)
    resumed = training.train(directory, updates=2, resume=True, keep_resume=keep_resume, **recipe)
    assert_exact_tree(resumed.state_dict(), checkpoint['model'])
    assert (directory/'metrics.jsonl').read_bytes() == metrics_before
    assert [row['update'] for row in read_metrics(directory)] == [1, 2]
    deployed, _ = load_actor(directory/'actor.pt')
    assert_exact_tree(deployed.state_dict(), expected_actor.state_dict())
    completed = json.loads((directory/'run.json').read_text())
    assert completed['status'] == 'completed'
    assert 'error' not in completed
    assert completed['completed_updates'] == completed['planned_updates'] == 2
    assert completed['initialization'] == manifest['initialization']
    assert completed['runtime'] == manifest['runtime']
    assert completed['actor_sha256'] == file_hash(directory/'actor.pt')
    assert completed['evaluation'] == 'evaluation.json'
    report = json.loads((directory/'evaluation.json').read_text())
    assert report['actor_sha256'] == completed['actor_sha256']
    assert report['split'] == 'test'
    assert len(report['cases']) == len(evaluation_calls) == 23
    assert {(row['passage_width'], row['jitter'], row['seed']) for row in evaluation_calls} == {
        (spec['passage_width'], spec['jitter'], spec['seed']) for spec in case_specs()}
    assert {row['seed'] for row in evaluation_calls} == set(TEST_SEEDS)
    if keep_resume:
        final_checkpoint = torch.load(checkpoint_path, weights_only=False)
        for key in ('update', 'model', 'optimizer', 'env', 'state', 'starts', 'torch_rng', 'cuda_rng'):
            assert_exact_tree(final_checkpoint[key], checkpoint[key])
    else:
        assert not (directory/'.work').exists()


def test_resume_rejects_changed_configuration_seed_and_source_without_rewriting_evidence(tmp_path, monkeypatch):
    recipe = small_recipe()
    directory = tmp_path/'strict-resume'
    training.train(directory, updates=1, keep_resume=True, **recipe)
    original_files = file_snapshot(directory)
    changed = {**recipe, 'scenario': replace(recipe['scenario'], max_jitter=.04)}
    with pytest.raises(ValueError, match='identical G config and source'):
        training.train(directory, updates=2, resume=True, **changed)
    assert file_snapshot(directory) == original_files
    changed = {**recipe, 'seed': recipe['seed']+1}
    with pytest.raises(ValueError, match='identical G config and source'):
        training.train(directory, updates=2, resume=True, **changed)
    assert file_snapshot(directory) == original_files
    monkeypatch.setattr(training, 'source_hash', lambda: 'changed-source-identity')
    with pytest.raises(ValueError, match='identical G config and source'):
        training.train(directory, updates=2, resume=True, **recipe)
    assert file_snapshot(directory) == original_files


def test_existing_directory_is_never_overwritten_and_resume_requires_retained_checkpoint(tmp_path):
    directory = tmp_path/'unrelated-work'
    directory.mkdir()
    (directory/'sentinel.txt').write_text('existing work must survive')
    original_files = file_snapshot(directory)
    with pytest.raises(FileExistsError, match='existing evidence'):
        training.train(directory, updates=1, **small_recipe())
    assert file_snapshot(directory) == original_files
    completed = tmp_path/'completed-run'
    training.train(completed, updates=1, **small_recipe())
    original_files = file_snapshot(completed)
    with pytest.raises(FileExistsError, match='existing evidence'):
        training.train(completed, updates=2, **small_recipe())
    with pytest.raises(FileNotFoundError, match='keep-resume'):
        training.train(completed, updates=2, resume=True, **small_recipe())
    assert file_snapshot(completed) == original_files


def test_unsupported_v1_checkpoint_is_rejected_without_modification(tmp_path):
    path = tmp_path/'unsupported-actor.pt'
    torch.save({'schema': 'afor_local_v1', 'kind': 'actor'}, path)
    before = path.read_bytes()
    with pytest.raises(ValueError, match='actor checkpoint'):
        load_actor(path)
    assert path.read_bytes() == before
