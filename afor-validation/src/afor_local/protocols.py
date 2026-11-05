"""Fresh, disjoint splits for the three-formation graph-policy study."""

FORMATIONS = ('line', 'circle', 'wedge')
TRAINING_SEEDS = (11, 23, 37, 51, 67)
VALIDATION_SEEDS = (172001, 172002)
TEST_SEEDS = (183001, 183002, 183003)
ANALYSIS_SEEDS = (184001, 184002, 184003)
UPDATES = 500
VALIDATION_EPISODES = 24
TEST_EPISODES = 32
VALIDATION_JITTERS = (0., .05, .10, .15)
TEST_JITTERS = (0., .05, .10, .15, .20)
TRANSFER_WIDTHS = (1.1, 1.3, 1.4, 1.5, 1.6)


def case_specs(passage_width=1.2, *, episodes=TEST_EPISODES, seeds=TEST_SEEDS):
    """Same 23 paired test conditions for every formation and training seed.

    Transfer uses the first test seed; the nominal width is already covered by
    the main grid. Each condition records its family to keep summaries separate.
    """
    if type(episodes) is not int or episodes < 1 or not seeds:
        raise ValueError('Test cases require positive integer episodes and at least one seed')
    cases = []
    for jitter in TEST_JITTERS:
        family = ('nominal_regression' if jitter == 0 else
                  'stronger_perturbation' if jitter == .20 else 'main_challenge')
        cases.extend({'passage_width': passage_width, 'jitter': jitter, 'seed': seed,
                      'episodes': episodes, 'family': family} for seed in seeds)
    cases.extend({'passage_width': width, 'jitter': 0., 'seed': seeds[0],
                  'episodes': episodes, 'family': 'width_transfer'}
                 for width in TRANSFER_WIDTHS if width != passage_width)
    cases.extend({'passage_width': 1.1, 'jitter': .15, 'seed': seed,
                  'episodes': episodes, 'family': 'joint_shift'} for seed in seeds)
    return cases
