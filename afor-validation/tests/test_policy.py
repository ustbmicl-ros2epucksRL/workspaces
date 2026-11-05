"""G decision paths, causal state, initialization, and checkpoint contracts."""
import hashlib
import json
import torch
import pytest
from afor_local.config import PolicyConfig
from afor_local.policy import LocalActor, ActorCritic, latent_log_prob
from afor_local.checkpoints import SCHEMA, VERSION, actor_payload, export_actor, load_actor, export_scripted_actor
from afor_local.critic import LocalCritic


def observation(shape=(6,), k=4, seed=14):
    rng = torch.Generator().manual_seed(seed)
    return {'ego': torch.randn(*shape, 5, generator=rng),
            'scan': torch.rand(*shape, 32, generator=rng),
            'neighbours': torch.randn(*shape, k, 8, generator=rng),
            'mask': torch.rand(*shape, k, generator=rng) > .2}


def grad_norm(module):
    return sum(p.grad.abs().sum().item() for p in module.parameters() if p.grad is not None)


def test_exact_counts_and_active_policy_gradients():
    model = ActorCritic(seed=72)
    actor = model.actor
    assert actor.parameter_counts() == {'training': 42845, 'decision': 42845}
    state = actor.initial_state((6,))
    losses = []
    for t in range(3):
        features, state, attention = actor.features_with_attention(observation(seed=t), state)
        assert attention.shape == (6, 4)
        latent = torch.linspace(-.4, .7, 12).reshape(6, 2)
        losses.append(-latent_log_prob(actor.distribution(features), latent).mean())
    torch.stack(losses).mean().backward()
    for module in (actor.ego, actor.compensation, actor.graph.peer, actor.graph.score,
                   actor.fuse, actor.cell, actor.mean):
        assert grad_norm(module) > 0
    assert all(torch.isfinite(p.grad).all() for p in actor.parameters() if p.grad is not None)
    assert all(p.grad is None for p in model.critic.parameters())


def critic_digest(model):
    digest = hashlib.sha256()
    for module in (model.critic, model.value):
        for value in module.state_dict().values():
            digest.update(value.cpu().numpy().tobytes())
    return digest.hexdigest()


def test_initialization_is_deterministic_with_independent_critic_and_preserves_rng():
    before = torch.random.get_rng_state().clone()
    first, repeated = ActorCritic(seed=321), ActorCritic(seed=321)
    assert torch.equal(before, torch.random.get_rng_state())
    assert isinstance(first.critic, LocalCritic)
    assert critic_digest(first) == critic_digest(repeated)
    assert critic_digest(ActorCritic(seed=322)) != critic_digest(first)
    assert all(torch.equal(value, repeated.state_dict()[name]) for name, value in first.state_dict().items())
    # Task-value regression cannot update the policy network.
    _, value_features, _ = first.features(observation(), first.initial_state((6,)))
    first.value(value_features).square().mean().backward()
    assert grad_norm(first.critic) > 0
    assert all(parameter.grad is None for parameter in first.actor.parameters())


def test_reset_precedes_attention_and_memory():
    actor = ActorCritic(seed=12).actor
    obs = observation()
    reset = torch.ones(6, dtype=torch.bool)
    expected = actor.features_with_attention(obs, actor.initial_state((6,)))
    actual = actor.features_with_attention(obs, torch.full((6, 64), float('nan')), reset)
    for a, b in zip(actual, expected):
        torch.testing.assert_close(a, b, rtol=0, atol=0)
    partial = torch.tensor([True, False, True, False, True, False])
    state = torch.randn(6, 64)
    manually_reset = torch.where(partial[:, None], 0., state)
    for a, b in zip(actor(obs, state, partial), actor(obs, manually_reset)):
        torch.testing.assert_close(a, b, rtol=0, atol=0)


def test_locality_masking_permutation_and_empty_neighbours():
    actor = ActorCritic(seed=11).actor
    obs, state = observation(), torch.randn(6, 64)
    action, memory = actor(obs, state)
    permutation = torch.tensor([3, 1, 0, 2])
    permuted = {**obs, 'neighbours': obs['neighbours'][:, permutation], 'mask': obs['mask'][:, permutation]}
    for expected, actual in zip((action, memory), actor(permuted, state)):
        torch.testing.assert_close(expected, actual, atol=1e-7, rtol=1e-6)
    nan_padding = {**obs, 'neighbours': torch.where(obs['mask'][..., None], obs['neighbours'], float('nan'))}
    for expected, actual in zip((action, memory), actor(nan_padding, state)):
        torch.testing.assert_close(expected, actual, rtol=0, atol=0)
    padded = {**obs, 'neighbours': torch.cat((obs['neighbours'], torch.full((6, 3, 8), float('nan'))), 1),
              'mask': torch.cat((obs['mask'], torch.zeros(6, 3, dtype=torch.bool)), 1)}
    for expected, actual in zip((action, memory), actor(padded, state)):
        torch.testing.assert_close(expected, actual, atol=1e-7, rtol=1e-6)
    all_masked = {**obs, 'neighbours': torch.full_like(obs['neighbours'], float('nan')),
                  'mask': torch.zeros_like(obs['mask'])}
    empty = {**obs, 'neighbours': obs['neighbours'][:, :0], 'mask': obs['mask'][:, :0]}
    for expected, actual in zip(actor(all_masked, state), actor(empty, state)):
        assert torch.isfinite(expected).all() and torch.isfinite(actual).all()
        torch.testing.assert_close(expected, actual, rtol=0, atol=0)
    attention = actor.features_with_attention(obs, state)[2]
    torch.testing.assert_close(attention.sum(-1), obs['mask'].any(-1).float())
    assert (attention[~obs['mask']] == 0).all()
    assert actor.features_with_attention(empty, state)[2].shape == (6, 0)
    changed = {key: value.clone() for key, value in obs.items()}
    changed['ego'][0] += 2
    changed['scan'][0] *= .3
    changed['neighbours'][0] += 10
    state_changed = state.clone()
    state_changed[0] += 3
    for expected, actual in zip((action, memory), actor(changed, state_changed)):
        torch.testing.assert_close(expected[1:], actual[1:], rtol=0, atol=0)


def test_checkpoint_and_script_roundtrip_dynamic_batch_and_empty_k(tmp_path):
    model = ActorCritic(seed=6)
    path = tmp_path / 'actor.pt'
    export_actor(model, path, .05)
    loaded, dt = load_actor(path)
    assert dt == .05 and loaded.config == model.actor.config
    payload = torch.load(path, weights_only=True)
    assert payload['schema'] == SCHEMA and payload['version'] == VERSION and payload['kind'] == 'actor'
    script_path = tmp_path / 'actor.ts'
    export_scripted_actor(model.actor, script_path)
    extra = {'metadata.json': ''}
    scripted = torch.jit.load(str(script_path), _extra_files=extra)
    metadata = json.loads(extra['metadata.json'])
    assert metadata['policy'] == {'hidden': 64, 'capacity_width': 24}
    assert metadata['interface']['inputs'] == ['ego', 'scan', 'neighbours', 'mask', 'state']
    for shape in ((), (1,), (3,), (2, 3)):
        for k in (0, 1, 5):
            obs = observation(shape, k)
            state = loaded.initial_state(shape)
            eager = model.actor(obs, state)
            restored = loaded(obs, state)
            args = [obs[key] for key in ('ego', 'scan', 'neighbours', 'mask')] + [state]
            actual = scripted(*args)
            for a, b, c in zip(eager, restored, actual):
                torch.testing.assert_close(a, b, rtol=0, atol=0)
                torch.testing.assert_close(a, c, rtol=0, atol=0)


@pytest.mark.parametrize('field,value', [('schema', 'afor_local_brg_v2'), ('version', 2), ('kind', 'trainer'),
                                        ('dt', float('nan')), ('dt', True), ('policy', {'hidden': 32}),
                                        ('interface', {}), ('state_dict', {'invalid': torch.tensor(float('nan'))})])
def test_checkpoint_rejects_invalid_metadata_and_weights(tmp_path, field, value):
    path = tmp_path / 'actor.pt'
    export_actor(ActorCritic(seed=1), path, .05)
    payload = torch.load(path, weights_only=True)
    payload[field] = value
    torch.save(payload, path)
    with pytest.raises(ValueError):
        load_actor(path)


def test_retained_payload_owns_weights_without_mutating_model_or_rng():
    actor = ActorCritic(seed=8).actor
    rng = torch.get_rng_state().clone()
    payload = actor_payload(actor, .05)
    assert actor.training
    assert torch.equal(rng, torch.get_rng_state())
    retained = payload['state_dict']['mean.weight'].clone()
    with torch.no_grad():
        actor.mean.weight.add_(1.)
    assert torch.equal(payload['state_dict']['mean.weight'], retained)


def test_actor_critic_forward_keeps_training_signature():
    model = ActorCritic(seed=8)
    obs = observation()
    state = model.initial_state((6,))
    assert state.shape == (6, 128)
    features, critic_features, next_state = model.features(obs, state)
    assert features.shape == critic_features.shape == (6, 64)
    assert next_state.shape == state.shape
    action, latent, logprob, value, memory = model(obs, state, deterministic=True)
    torch.testing.assert_close(action, latent.tanh())
    assert logprob.shape == value.shape == (6,)
    assert memory.shape == state.shape


@pytest.mark.parametrize('options', ({'hidden': 32}, {'capacity_width': 80}))
def test_architecture_dimensions_are_fixed(options):
    with pytest.raises(ValueError):
        PolicyConfig(**options)
