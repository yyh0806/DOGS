from tools.topic_rate_gate import evaluate_receive_times


def test_topic_rate_gate_accepts_rate_at_threshold():
    passed, rate, failure = evaluate_receive_times(
        [0.0, 0.1, 0.2, 0.3], minimum_hz=10.0, minimum_samples=4
    )
    assert passed
    assert rate >= 10.0
    assert failure is None


def test_topic_rate_gate_rejects_missing_and_slow_streams():
    assert evaluate_receive_times(
        [0.0], minimum_hz=1.0, minimum_samples=2
    )[2] == "insufficient_samples"
    assert evaluate_receive_times(
        [0.0, 1.0, 2.0], minimum_hz=2.0, minimum_samples=3
    )[2] == "rate_below_minimum"
