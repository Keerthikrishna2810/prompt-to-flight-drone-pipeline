"""
Day 2 exit-criterion test: the validator must correctly accept/reject every
fixture, with a clear (non-empty) error message on every rejection.

Run with:  pytest test_validator.py -v
Or directly:  python3 test_validator.py
"""

import json
import pathlib

import pytest

from validator import SafetyConfig, validate_mission_json

FIXTURES_DIR = pathlib.Path(__file__).parent / "fixtures"

# Home position the fixtures were hand-written against — see fixtures/*.json
SAFETY_CFG = SafetyConfig(
    home_lat=10.5270,
    home_lon=76.2140,
    max_geofence_radius_m=200.0,
    min_alt_m=3.0,
    max_alt_m=25.0,
    max_speed_mps=8.0,
    max_leg_distance_m=150.0,
    loop_closure_tolerance_m=15.0,
)


def _load_fixtures():
    cases = []
    for path in sorted(FIXTURES_DIR.glob("*.json")):
        raw = json.loads(path.read_text())
        expect = raw.pop("_expect")
        raw.pop("_note", None)
        cases.append((path.name, raw, expect))
    return cases


@pytest.mark.parametrize("name,raw,expect", _load_fixtures())
def test_fixture(name, raw, expect):
    result = validate_mission_json(raw, SAFETY_CFG)

    if expect == "accept":
        assert result.ok, f"{name}: expected accept, got errors: {result.errors}"
    else:
        assert not result.ok, f"{name}: expected reject ({expect}), but was accepted"
        assert len(result.errors) > 0, f"{name}: rejected with no error message"


if __name__ == "__main__":
    passed, failed = 0, 0
    for name, raw, expect in _load_fixtures():
        result = validate_mission_json(raw, SAFETY_CFG)
        ok_as_expected = result.ok if expect == "accept" else not result.ok
        status = "PASS" if ok_as_expected else "FAIL"
        passed += ok_as_expected
        failed += not ok_as_expected
        print(f"[{status}] {name:35s} expect={expect:14s} ok={result.ok}  errors={result.errors}")
    print(f"\n{passed} passed, {failed} failed")
