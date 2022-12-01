import pytest

import georgio

from .locations import lon_lats

def test_bounding_box_for_point():
    distances = (100.0, 1000.0, 10000.0)

    for start_lon, start_lat in lon_lats:
        for distance in distances:
            west, south, east, north = georgio.bounding_box_for_point(start_lon, start_lat, distance)
            assert georgio.great_circle_distance(west, start_lat, start_lon, start_lat) == pytest.approx(distance, rel=0.01)
            assert georgio.great_circle_distance(start_lon, south, start_lon, start_lat) == pytest.approx(distance, rel=0.01)
            assert georgio.great_circle_distance(east, start_lat, start_lon, start_lat) == pytest.approx(distance, rel=0.01)
            assert georgio.great_circle_distance(start_lon, north, start_lon, start_lat) == pytest.approx(distance, rel=0.01)
