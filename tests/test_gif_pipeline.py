from pathlib import Path


def test_demo_gif_script_exists():
    script = Path("scripts/make_demo_gif.py")
    assert script.exists()
    assert "assets / \"demo.gif\"" in script.read_text(encoding="utf-8")
