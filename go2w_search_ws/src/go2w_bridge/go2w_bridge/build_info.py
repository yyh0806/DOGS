"""Immutable build identity shared by every NX runtime process."""

from __future__ import annotations

import os
from collections.abc import Mapping


MAX_RELEASE_ID_LENGTH = 64


def release_id(env: Mapping[str, object] | None = None) -> str:
    """Return the deployment-provided release identifier.

    Runtime code must not inspect Git because the NX release directory is an
    immutable artifact.  Deployment is responsible for setting the value.
    """

    source = os.environ if env is None else env
    value = str(source.get("GO2W_RELEASE_ID", "development")).strip()
    return value[:MAX_RELEASE_ID_LENGTH] or "development"

