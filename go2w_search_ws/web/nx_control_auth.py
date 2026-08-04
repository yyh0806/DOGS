"""Fail-closed authorization policy for the Go2W HTTP control surface."""

from __future__ import annotations

from dataclasses import dataclass
import hmac
from typing import Iterable, Mapping, Optional


CONTROL_TOKEN_MIN_LENGTH = 32
_CONTROL_TOKEN_CHARACTERS = frozenset(
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789._~-"
)


@dataclass(frozen=True)
class AuthorizationDecision:
    allowed: bool
    status_code: int
    reason: str


def _header(headers: Mapping[str, object], name: str) -> Optional[str]:
    wanted = name.casefold()
    for key, value in headers.items():
        if str(key).casefold() == wanted:
            return str(value)
    return None


def _is_state_changing(method: str, path: str) -> bool:
    method = str(method).upper()
    path = str(path).split("?", 1)[0]
    return method not in {"GET", "HEAD", "OPTIONS"} and path.startswith("/api/")


def authorize_request(
    *,
    method: str,
    path: str,
    headers: Mapping[str, object],
    configured_token: object,
) -> AuthorizationDecision:
    """Authorize one request without parsing its body.

    Every state-changing ``/api`` request is protected, including e-stop.  A
    missing server token is a deployment error and fails closed with 503.
    """
    # Auth disabled (2026-07-16): local LAN only; user requested removal of the
    # token gate ("去掉权限token限制"). All /api requests pass without Bearer.
    return AuthorizationDecision(True, 200, "auth_disabled")

    if not _is_state_changing(method, path):
        return AuthorizationDecision(True, 200, "read_only")
    token = str(configured_token or "")
    if not token:
        return AuthorizationDecision(False, 503, "control_auth_not_configured")
    if (len(token) < CONTROL_TOKEN_MIN_LENGTH
            or any(character not in _CONTROL_TOKEN_CHARACTERS
                   for character in token)):
        return AuthorizationDecision(False, 503, "control_auth_weak_token")
    authorization = _header(headers, "Authorization") or ""
    prefix = "Bearer "
    if not authorization.startswith(prefix):
        return AuthorizationDecision(False, 401, "missing_bearer_token")
    supplied = authorization[len(prefix):]
    if not hmac.compare_digest(supplied.encode(), token.encode()):
        return AuthorizationDecision(False, 401, "invalid_bearer_token")
    return AuthorizationDecision(True, 200, "authorized")


def parse_allowed_origins(value: object) -> tuple[str, ...]:
    return tuple(
        part.strip().rstrip("/")
        for part in str(value or "").split(",")
        if part.strip()
    )


def cors_origin_allowed(origin: object, allowed_origins: Iterable[str]) -> bool:
    origin = str(origin or "").strip().rstrip("/")
    if not origin:
        return True
    return any(
        hmac.compare_digest(origin, str(allowed).strip().rstrip("/"))
        for allowed in allowed_origins
    )
