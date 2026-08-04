"""Pure tests for the read-only FAST_LIO age acceptance gate."""

from __future__ import annotations

import ast
import importlib.util
from pathlib import Path

import pytest


PATH = Path(__file__).with_name("fastlio_latency_gate.py")


def _load():
    spec = importlib.util.spec_from_file_location("fastlio_latency_gate", PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_age_gate_accepts_fresh_monotonic_distribution():
    gate = _load()
    result = gate.evaluate_samples(
        [(10.0 + index * 0.1, age) for index, age in enumerate(
            [0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.21, 0.22, 0.23, 0.24]
        )],
        minimum_samples=10,
        max_median_age_sec=0.30,
        max_p95_age_sec=0.35,
    )
    assert result.passed is True
    assert result.failures == ()


@pytest.mark.parametrize(
    "samples, expected",
    [
        ([(1.0, 0.1)], "insufficient_samples"),
        ([(float(index), float("nan")) for index in range(10)], "nonfinite_age"),
        ([(10.0 - index, 0.1) for index in range(10)], "nonmonotonic_stamp"),
        ([(float(index), 0.31) for index in range(10)], "median_age"),
        (
            [(float(index), 0.1 if index < 9 else 0.8) for index in range(10)],
            "p95_age",
        ),
    ],
)
def test_age_gate_rejects_unsafe_samples(samples, expected):
    gate = _load()
    result = gate.evaluate_samples(
        samples,
        minimum_samples=10,
        max_median_age_sec=0.30,
        max_p95_age_sec=0.35,
    )
    assert result.passed is False
    assert expected in result.failures


def test_latency_gate_source_is_subscription_only():
    source = PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    attributes = {
        node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
    }
    assert "create_subscription" in attributes
    assert "create_publisher" not in attributes
    assert '"/Odometry"' in source
    assert "self._subscription = self.create_subscription" in source


def test_startup_warmup_is_excluded_but_steady_state_is_fully_evaluated():
    gate = _load()
    startup = [(1.0, 1.20), (1.1, 0.92)]
    steady = [(2.0 + index * 0.1, 0.08) for index in range(100)]
    window = gate._post_warmup_samples(
        startup + steady,
        warmup_samples=2,
        sample_count=100,
    )
    assert window == steady
    result = gate.evaluate_samples(
        window,
        minimum_samples=90,
        max_median_age_sec=0.30,
        max_p95_age_sec=0.35,
    )
    assert result.passed is True
