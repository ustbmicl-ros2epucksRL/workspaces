"""Real PPO integration: selected/terminal exports, RNG and exact recovery."""
from dataclasses import replace
import json

import pytest
import torch

from afor_local.artifacts import file_hash, tensor_state_hash
from afor_local.checkpoints import load_actor
from afor_local.config import SelectionConfig
from tests.test_training import training, small_recipe, assert_exact_tree, file_snapshot


def recipe():
    result = small_recipe()
    result['selection'] = SelectionConfig(enabled=True, interval=1, episodes=1)
    return result


def test_validation_selects_one_actor_and_preserves_terminal_training_rng(tmp_path):
    selected = tmp_path/'selected'
    kwargs = recipe()
    model = training.train(selected, updates=2, **kwargs)
    reference = small_recipe()
    control = training.train(tmp_path/'control', updates=2, **reference)
    assert_exact_tree(model.state_dict(), control.state_dict())
    assert {p.name for p in selected.iterdir()} == {'actor.pt', 'final_actor.pt', 'metrics.jsonl', 'run.json'}
    run = json.loads((selected/'run.json').read_text())
    # Both checkpoints time out on this deliberately 11-step task. Same scores
    # retain the first checkpoint, demonstrating selection rather than aliasing.
    assert run['selection']['selected_update'] == 1
    assert run['selection']['check_count'] == 2
    assert run['actor_sha256'] == file_hash(selected/'actor.pt')
    assert run['final_actor_sha256'] == file_hash(selected/'final_actor.pt')
    actor, _ = load_actor(selected/'actor.pt')
    final, _ = load_actor(selected/'final_actor.pt')
    assert tensor_state_hash(actor.state_dict()) == run['selection']['best_actor_state_sha256']
    assert_exact_tree(final.state_dict(), model.actor.state_dict())
    assert any(not torch.equal(w, final.state_dict()[name]) for name, w in actor.state_dict().items())


def test_selected_and_endpoint_evaluations_share_all_cases_and_keep_only_five_files(tmp_path):
    kwargs = recipe()
    kwargs.update(evaluate_final=True, eval_episodes=1)
    training.train(tmp_path/'run', updates=2, **kwargs)
    folder = tmp_path/'run'
    assert {p.name for p in folder.iterdir()} == {'actor.pt', 'final_actor.pt', 'metrics.jsonl', 'run.json', 'evaluation.json'}
    run = json.loads((folder/'run.json').read_text())
    evaluation = json.loads((folder/'evaluation.json').read_text())
    assert evaluation['selection'] == run['selection']
    assert evaluation['actor_sha256'] == run['actor_sha256']
    assert evaluation['endpoint']['actor_sha256'] == run['final_actor_sha256']
    assert evaluation['endpoint']['update'] == 2
    assert [c['case_hash'] for c in evaluation['cases']] == [c['case_hash'] for c in evaluation['endpoint']['cases']]
    assert len(evaluation['cases']) == 23


def test_resume_restores_best_payload_history_and_terminal_optimizer(tmp_path):
    kwargs = recipe()
    split, direct = tmp_path/'split', tmp_path/'direct'
    training.train(split, updates=1, keep_resume=True, **kwargs)
    training.train(split, updates=2, resume=True, keep_resume=True, **kwargs)
    training.train(direct, updates=2, keep_resume=True, **kwargs)
    a = torch.load(split/'.work/latest.pt', weights_only=False)
    b = torch.load(direct/'.work/latest.pt', weights_only=False)
    for key in ('model', 'optimizer', 'env', 'state', 'starts', 'torch_rng', 'selection'):
        assert_exact_tree(a[key], b[key])
    for name in ('actor.pt', 'final_actor.pt'):
        first, _ = load_actor(split/name)
        second, _ = load_actor(direct/name)
        assert_exact_tree(first.state_dict(), second.state_dict())


def test_extending_from_extra_off_schedule_validation_is_rejected_without_rewriting(tmp_path):
    kwargs = recipe()
    kwargs['selection'] = replace(kwargs['selection'], interval=2)
    folder = tmp_path/'run'
    training.train(folder, updates=1, keep_resume=True, **kwargs)
    before = file_snapshot(folder)
    with pytest.raises(ValueError, match='off-schedule'):
        training.train(folder, updates=2, resume=True, keep_resume=True, **kwargs)
    assert before == file_snapshot(folder)
