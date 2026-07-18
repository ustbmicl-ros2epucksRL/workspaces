import threading

from robosar_emergency_response.global_info import GlobalInfoStateBuffer


def test_runtime_state_snapshot_is_stable_under_concurrent_odom_updates():
    state = GlobalInfoStateBuffer()
    stop = threading.Event()

    def update_odometry():
        index = 0
        while not stop.is_set():
            state.update_robot("tb_{}".format(index % 4), index)
            index += 1

    worker = threading.Thread(target=update_odometry)
    worker.start()
    try:
        for _ in range(2000):
            robots, model_states, active_task = state.snapshot()
            # Iterating the detached snapshot must never race the writer.
            list(robots.items())
            assert model_states is None
            assert active_task is None
    finally:
        stop.set()
        worker.join(timeout=1.0)

    robots, _, _ = state.snapshot()
    state.update_robot("tb_new", 123)
    assert "tb_new" not in robots
