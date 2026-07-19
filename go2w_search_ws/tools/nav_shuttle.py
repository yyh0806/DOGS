#!/usr/bin/env python3
"""Run success-gated point-navigation shuttles through the Go2W HTTP API.

The command is a dry run unless ``--execute`` is supplied. Each goal is sent
only after the preceding goal's matching navigation generation succeeds.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Callable, Iterable, Mapping, Optional, Protocol, Sequence
import urllib.error
import urllib.parse
import urllib.request


TERMINAL_STATUSES = frozenset({
    "succeeded", "aborted", "canceled", "failed", "rejected", "timed_out",
})
MAX_TRIPS = 100


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float
    frame_id: str = "map"

    def payload(self) -> dict:
        return asdict(self)


class Transport(Protocol):
    def post(self, path: str, payload: Mapping[str, object]) -> dict: ...

    def get(self, path: str) -> dict: ...


class TransportError(RuntimeError):
    pass


def _finite_number(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        normalized = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be a finite number")
    return normalized


def _point(value: object, name: str) -> tuple[float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"{name} must contain exactly x and y")
    return (
        _finite_number(value[0], f"{name}.x"),
        _finite_number(value[1], f"{name}.y"),
    )


def _trip_count(value: object) -> int:
    if isinstance(value, bool):
        raise ValueError("trips must be a positive bounded integer")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("trips must be a positive bounded integer") from exc
    if (not math.isfinite(numeric) or not numeric.is_integer()
            or numeric < 1 or numeric > MAX_TRIPS):
        raise ValueError("trips must be a positive bounded integer")
    return int(numeric)


def build_shuttle_goals(
    start: object = (0.0, 0.0),
    end: object = (20.0, 0.0),
    *,
    trips: object = 3,
) -> list[Goal]:
    """Build end/start pairs for the requested number of round trips."""
    start_x, start_y = _point(start, "start")
    end_x, end_y = _point(end, "end")
    count = _trip_count(trips)
    if math.hypot(end_x - start_x, end_y - start_y) <= 1e-9:
        raise ValueError("start and end must be different points")
    outbound_yaw = math.atan2(end_y - start_y, end_x - start_x)
    return_yaw = math.atan2(start_y - end_y, start_x - end_x)
    goals = []
    for _ in range(count):
        goals.append(Goal(end_x, end_y, outbound_yaw))
        goals.append(Goal(start_x, start_y, return_yaw))
    return goals


class HttpTransport:
    def __init__(
        self,
        base_url: str,
        *,
        timeout: float = 3.0,
        token: Optional[str] = None,
    ) -> None:
        parsed = urllib.parse.urlsplit(str(base_url).rstrip("/"))
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise ValueError("base_url must be an HTTP(S) origin")
        self._base_url = urllib.parse.urlunsplit(
            (parsed.scheme, parsed.netloc, parsed.path.rstrip("/"), "", "")
        )
        self._timeout = _finite_number(timeout, "http_timeout")
        if self._timeout <= 0.0:
            raise ValueError("http_timeout must be positive")
        self._token = str(token or "").strip()

    def get(self, path: str) -> dict:
        return self._request("GET", path, None)

    def post(self, path: str, payload: Mapping[str, object]) -> dict:
        return self._request("POST", path, payload)

    def _request(
        self,
        method: str,
        path: str,
        payload: Optional[Mapping[str, object]],
    ) -> dict:
        if not str(path).startswith("/"):
            raise TransportError("API path must start with /")
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Accept": "application/json"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        if self._token:
            headers["Authorization"] = f"Bearer {self._token}"
        request = urllib.request.Request(
            f"{self._base_url}{path}", data=data, headers=headers, method=method
        )
        try:
            with urllib.request.urlopen(request, timeout=self._timeout) as response:
                body = response.read()
        except urllib.error.HTTPError as exc:
            body = exc.read()
            detail = _decode_json(body)
            reason = detail.get("reason") or detail.get("msg") or str(exc)
            raise TransportError(f"HTTP {exc.code}: {reason}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise TransportError(str(exc)) from exc
        result = _decode_json(body)
        if not isinstance(result, dict):
            raise TransportError("API response must be a JSON object")
        return result


def _decode_json(body: bytes) -> dict:
    try:
        value = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise TransportError("API returned invalid JSON") from exc
    if not isinstance(value, dict):
        raise TransportError("API response must be a JSON object")
    return value


class ShuttleRunner:
    def __init__(
        self,
        transport: Transport,
        *,
        poll_interval: float = 0.5,
        leg_timeout: float = 240.0,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._transport = transport
        self._poll_interval = _finite_number(poll_interval, "poll_interval")
        self._leg_timeout = _finite_number(leg_timeout, "leg_timeout")
        if self._poll_interval < 0.0:
            raise ValueError("poll_interval must be non-negative")
        if self._leg_timeout <= 0.0:
            raise ValueError("leg_timeout must be positive")
        self._monotonic = monotonic
        self._sleep = sleep

    def run(self, goals: Iterable[Goal]) -> dict:
        ordered = list(goals)
        completed = 0
        for leg_number, goal in enumerate(ordered, start=1):
            if not isinstance(goal, Goal):
                raise ValueError("goals must contain Goal values")
            try:
                submitted = self._transport.post("/api/navigate", goal.payload())
            except Exception as exc:
                self._safe_stop()
                return self._failure(
                    completed, leg_number, None, "transport_error", str(exc)
                )
            if not submitted.get("ok"):
                self._safe_stop()
                return self._failure(
                    completed,
                    leg_number,
                    submitted.get("generation"),
                    "rejected",
                    str(submitted.get("reason") or submitted.get("msg")
                        or "navigate_rejected"),
                )
            try:
                generation = int(submitted["generation"])
            except (KeyError, TypeError, ValueError):
                self._safe_stop()
                return self._failure(
                    completed, leg_number, None, "invalid_response",
                    "navigate response has no valid generation",
                )
            deadline = self._monotonic() + self._leg_timeout
            while self._monotonic() < deadline:
                try:
                    status_payload = self._transport.get("/api/status")
                except Exception as exc:
                    self._safe_stop()
                    return self._failure(
                        completed, leg_number, generation, "transport_error", str(exc)
                    )
                state = status_payload.get("point_nav")
                if isinstance(state, dict):
                    state_generation = _optional_int(state.get("generation"))
                    status = str(state.get("status") or "").lower()
                    if state_generation is not None and state_generation > generation:
                        self._safe_stop()
                        return self._failure(
                            completed, leg_number, generation, "superseded",
                            "navigation generation was replaced",
                        )
                    if state_generation == generation and status == "succeeded":
                        completed += 1
                        break
                    if state_generation == generation and status in TERMINAL_STATUSES:
                        reason = str(state.get("reason") or status)
                        self._safe_stop()
                        return self._failure(
                            completed, leg_number, generation, status, reason
                        )
                if self._poll_interval:
                    self._sleep(self._poll_interval)
            else:
                self._safe_stop()
                return self._failure(
                    completed, leg_number, generation, "timed_out", "leg_timeout"
                )
        return {"ok": True, "legs_completed": completed, "legs_total": len(ordered)}

    def _safe_stop(self) -> None:
        try:
            self._transport.post("/api/stop", {})
        except Exception:
            pass

    @staticmethod
    def _failure(
        completed: int,
        leg_number: int,
        generation: object,
        status: str,
        reason: str,
    ) -> dict:
        return {
            "ok": False,
            "legs_completed": completed,
            "failed_leg": leg_number,
            "generation": generation,
            "status": status,
            "reason": reason,
        }


def _optional_int(value: object) -> Optional[int]:
    if isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Success-gated Go2W point-navigation shuttle"
    )
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--end-x", type=float, default=20.0)
    parser.add_argument("--end-y", type=float, default=0.0)
    parser.add_argument("--trips", type=int, default=3)
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--token", default=None)
    parser.add_argument("--leg-timeout", type=float, default=240.0)
    parser.add_argument("--poll-interval", type=float, default=0.5)
    parser.add_argument("--http-timeout", type=float, default=3.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="send goals to the robot; omitted means print the route only",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        goals = build_shuttle_goals(
            (args.start_x, args.start_y),
            (args.end_x, args.end_y),
            trips=args.trips,
        )
        if not args.execute:
            print(json.dumps({
                "mode": "dry-run",
                "trips": args.trips,
                "goals": [goal.payload() for goal in goals],
            }, ensure_ascii=False, indent=2))
            return 0
        transport = HttpTransport(
            args.base_url, timeout=args.http_timeout, token=args.token
        )
        runner = ShuttleRunner(
            transport,
            poll_interval=args.poll_interval,
            leg_timeout=args.leg_timeout,
        )
        result = runner.run(goals)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get("ok") else 1
    except (ValueError, TransportError) as exc:
        print(json.dumps({"ok": False, "reason": str(exc)}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
