import importlib.util
import os
from pathlib import Path
import re

import pytest


MODULE_PATH = Path(__file__).with_name("generate_control_token.py")


def load_module():
    spec = importlib.util.spec_from_file_location("generate_control_token", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_create_control_token_is_url_safe_private_and_not_printed(tmp_path, capsys):
    module = load_module()
    path = tmp_path / "control-token.txt"

    result = module.create_control_token(path)

    token = path.read_text(encoding="ascii").strip()
    assert result == path
    assert re.fullmatch(r"[A-Za-z0-9_-]{43}", token)
    assert token not in capsys.readouterr().out
    if os.name != "nt":
        assert path.stat().st_mode & 0o777 == 0o600


def test_create_control_token_never_overwrites_existing_secret(tmp_path):
    module = load_module()
    path = tmp_path / "control-token.txt"
    path.write_text("keep-this-token\n", encoding="ascii")

    with pytest.raises(FileExistsError):
        module.create_control_token(path)

    assert path.read_text(encoding="ascii") == "keep-this-token\n"


def test_cli_reports_path_without_disclosing_token(tmp_path, capsys):
    module = load_module()
    path = tmp_path / "control-token.txt"

    assert module.main([str(path)]) == 0

    output = capsys.readouterr().out
    token = path.read_text(encoding="ascii").strip()
    assert str(path) in output
    assert token not in output
