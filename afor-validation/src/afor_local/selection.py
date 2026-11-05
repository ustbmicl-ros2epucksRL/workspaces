"""Validation-only, paired checkpoint selection with exact RNG isolation.

No evaluation trace or intermediate actor is persisted. The tracker retains one
CPU actor payload; training's existing recovery checkpoint stores its state.
"""
from contextlib import contextmanager
import copy
from dataclasses import asdict
import math
import random

import numpy as np

import torch

from .checkpoints import SCHEMA, actor_payload
from .artifacts import tensor_state_hash
from .config import SelectionConfig
from .evaluate import build_case_set, evaluate_case


@contextmanager
def _preserve_evaluation_state(actor=None):
    python_rng, numpy_rng = random.getstate(), np.random.get_state()
    cpu_rng = torch.get_rng_state().clone()
    cuda_rng = torch.cuda.get_rng_state_all() if torch.cuda.is_initialized() else None
    modes = [(module, module.training) for module in actor.modules()] if actor is not None else []
    try:
        if actor is not None:
            actor.eval()
        with torch.no_grad():
            yield
    finally:
        for module, training in modes:
            module.training = training
        random.setstate(python_rng)
        np.random.set_state(numpy_rng)
        torch.set_rng_state(cpu_rng)
        if cuda_rng is not None:
            torch.cuda.set_rng_state_all(cuda_rng)


def build_validation_cases(dt, task, config):
    """Build actor-independent fixtures before training, preserving global RNG."""
    if not config.enabled:
        return ()
    with _preserve_evaluation_state():
        return tuple(build_case_set(dt, task.formation, task.passage_width, jitter, seed,
                                    config.episodes, task.horizon)
                     for jitter in config.jitters for seed in config.validation_seeds)


def _case_identities(case_sets):
    return [{'seed': case['seed'], 'jitter': case['jitter'], 'case_hash': case['case_hash']}
            for case in case_sets]


def selection_key(score):
    """Nominal success, challenge success, then lower penalized challenge time."""
    for prefix in ('nominal', 'challenge'):
        successes, episodes = score[f'{prefix}_successes'], score[f'{prefix}_episodes']
        if (type(successes) is not int or type(episodes) is not int or episodes < 1
                or not 0 <= successes <= episodes):
            raise ValueError('Selection scores require valid integer episode/success counts')
    duration = score['mean_failure_penalized_duration']
    if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration < 0:
        raise ValueError('Selection duration must be finite and nonnegative')
    return score['nominal_successes'], score['challenge_successes'], -float(duration)


def validate_current_actor(actor, dt, task, config=SelectionConfig(), *, case_sets=None):
    """Return compact validation scores; no pressure/transfer/test seeds enter."""
    if not config.enabled:
        raise ValueError('Checkpoint validation requires enabled selection')
    cases = build_validation_cases(dt, task, config) if case_sets is None else tuple(case_sets)
    expected = [(jitter, seed) for jitter in config.jitters for seed in config.validation_seeds]
    if [(case['jitter'], case['seed']) for case in cases] != expected:
        raise ValueError('Selection fixtures must contain exactly the ordered registered validation cases')
    nominal_successes = nominal_episodes = challenge_successes = challenge_episodes = 0
    challenge_duration = 0.
    compact = []
    with _preserve_evaluation_state(actor):
        for fixture in cases:
            report, _ = evaluate_case(actor, dt, task.formation, task.passage_width,
                                       fixture['jitter'], fixture['seed'], config.episodes,
                                       task.horizon, trace=False, case_set=fixture)
            successes, episodes = report['successes'], report['episodes']
            duration = report['mean_failure_penalized_duration']
            # Also reject malformed evaluator output in a replaced/custom
            # evaluator before it can select or export an actor.
            selection_key({'nominal_successes': successes, 'nominal_episodes': episodes,
                           'challenge_successes': successes, 'challenge_episodes': episodes,
                           'mean_failure_penalized_duration': duration})
            if episodes != config.episodes or report['case_hash'] != fixture['case_hash']:
                raise ValueError('Validation report differs from its paired fixture')
            if fixture['jitter'] == 0:
                nominal_successes += successes
                nominal_episodes += episodes
            else:
                challenge_successes += successes
                challenge_episodes += episodes
                challenge_duration += duration*episodes
            compact.append({'seed': fixture['seed'], 'jitter': fixture['jitter'],
                            'case_hash': report['case_hash'], 'episodes': episodes,
                            'successes': successes, 'mean_failure_penalized_duration': duration})
    return {'nominal_successes': nominal_successes, 'nominal_episodes': nominal_episodes,
            'challenge_successes': challenge_successes, 'challenge_episodes': challenge_episodes,
            'mean_failure_penalized_duration': challenge_duration/challenge_episodes,
            'validation_seeds': list(config.validation_seeds),
            'case_hashes': _case_identities(cases), 'cases': compact}


class BestTracker:
    """One selected CPU actor, compact scores and an exact resumable schedule."""
    def __init__(self, dt, task, config=SelectionConfig(), *, case_sets=None):
        self.dt, self.task, self.config = float(dt), task, config
        self.case_sets = (build_validation_cases(dt, task, config) if case_sets is None else tuple(case_sets))
        self.best_actor_payload = None
        self.best_update = None
        self.best_score = None
        self.best_actor_state_sha256 = None
        self.last_update = 0
        self.check_count = 0
        self.history = []

    def due(self, update, total_updates):
        return self.config.enabled and update > 0 and (update % self.config.interval == 0 or update == total_updates)

    def observe(self, actor, update):
        if not self.config.enabled:
            raise ValueError('BestTracker is disabled')
        if type(update) is not int or update <= self.last_update:
            raise ValueError('Validation observations require strictly increasing positive updates')
        score = validate_current_actor(actor, self.dt, self.task, self.config, case_sets=self.case_sets)
        improved = self.best_score is None or selection_key(score) > selection_key(self.best_score)
        if improved:
            payload = actor_payload(actor, self.dt)
            self.best_actor_payload, self.best_update, self.best_score = payload, update, copy.deepcopy(score)
            self.best_actor_state_sha256 = tensor_state_hash(payload['state_dict'])
        self.last_update = update
        self.check_count += 1
        record = {'update': update, 'check_count': self.check_count, 'improved': improved,
                  'selected_update': self.best_update, 'score': score,
                  'selected_actor_state_sha256': self.best_actor_state_sha256}
        self.history.append(copy.deepcopy(record))
        return record

    def summary(self):
        return {'enabled': self.config.enabled, 'config': asdict(self.config),
                'criterion': 'lexicographic nominal successes, challenge successes, negative penalized challenge duration',
                'interval': self.config.interval, 'episodes': self.config.episodes,
                'validation_seeds': list(self.config.validation_seeds), 'jitters': list(self.config.jitters),
                'ranking': ['nominal_successes', 'challenge_successes', '-mean_failure_penalized_duration'],
                'tie_break': 'earlier checkpoint', 'selected_update': self.best_update,
                'last_update': self.last_update, 'check_count': self.check_count,
                'best_actor_state_sha256': self.best_actor_state_sha256,
                'best_score': copy.deepcopy(self.best_score), 'case_hashes': _case_identities(self.case_sets),
                'history': copy.deepcopy(self.history)}

    def state_dict(self):
        return {'schema': 'afor_g_selection_v1', 'dt': self.dt, 'task': asdict(self.task),
                'config': asdict(self.config), 'case_hashes': _case_identities(self.case_sets),
                'best_actor_payload': copy.deepcopy(self.best_actor_payload),
                'best_update': self.best_update, 'best_score': copy.deepcopy(self.best_score),
                'best_actor_state_sha256': self.best_actor_state_sha256,
                'last_update': self.last_update, 'check_count': self.check_count,
                'history': copy.deepcopy(self.history)}

    def load_state_dict(self, state):
        if (state.get('schema') != 'afor_g_selection_v1' or state.get('dt') != self.dt
                or state.get('task') != asdict(self.task) or state.get('config') != asdict(self.config)
                or state.get('case_hashes') != _case_identities(self.case_sets)):
            raise ValueError('Selection resume requires identical configuration and paired validation cases')
        last, count, best = state.get('last_update'), state.get('check_count'), state.get('best_update')
        score, payload = state.get('best_score'), state.get('best_actor_payload')
        fingerprint = state.get('best_actor_state_sha256')
        history = state.get('history')
        if type(last) is not int or type(count) is not int or last < 0 or count < 0:
            raise ValueError('Invalid selection checkpoint schedule')
        if (not isinstance(history, list) or len(history) != count
                or any(type(row.get('update')) is not int for row in history)
                or [row['update'] for row in history] != sorted(set(row['update'] for row in history))
                or history and history[-1]['update'] != last):
            raise ValueError('Invalid selection checkpoint history')
        if count == 0:
            if last != 0 or best is not None or score is not None or payload is not None or fingerprint is not None:
                raise ValueError('Invalid empty selection checkpoint')
        else:
            if (type(best) is not int or not 0 < best <= last or not isinstance(score, dict)
                    or not isinstance(payload, dict) or payload.get('schema') != SCHEMA
                    or payload.get('kind') != 'actor' or payload.get('dt') != self.dt
                    or score.get('case_hashes') != _case_identities(self.case_sets)):
                raise ValueError('Invalid selected actor checkpoint')
            selection_key(score)
            if (score['nominal_episodes'] != len(self.config.validation_seeds)*self.config.episodes
                    or score['challenge_episodes'] != (len(self.config.jitters)-1)*len(self.config.validation_seeds)*self.config.episodes):
                raise ValueError('Selection checkpoint episode counts differ')
            weights = payload.get('state_dict')
            if not isinstance(weights, dict) or not all(isinstance(value, torch.Tensor) and torch.isfinite(value).all()
                                                       for value in weights.values()):
                raise ValueError('Invalid selected actor weights')
            if fingerprint != tensor_state_hash(weights):
                raise ValueError('Selected actor tensor fingerprint differs')
        self.best_actor_payload = copy.deepcopy(payload)
        if payload is not None:
            self.best_actor_payload['state_dict'] = {name: value.detach().cpu().clone()
                                                     for name, value in payload['state_dict'].items()}
        self.best_update, self.best_score = best, copy.deepcopy(score)
        self.best_actor_state_sha256 = fingerprint
        self.last_update, self.check_count = last, count
        self.history = copy.deepcopy(history)
