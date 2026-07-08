from pathlib import Path


def test_demo_gif_script_exists():
    script = Path("scripts/make_demo_gif.py")
    text = script.read_text(encoding="utf-8")
    assert script.exists()
    assert "assets / \"demo.gif\"" in text
    assert "results/videos" in text
    assert "wind_arrow" in text
    assert "P_safe" in text
    assert "RTH" in text or "RETURN-TO-HOME" in text
