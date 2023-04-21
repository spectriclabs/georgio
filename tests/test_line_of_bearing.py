from itertools import permutations
from timeit import timeit

import pytest

import geopy.distance
import georgio

from .locations import lon_lats

distances_meters = (100.0, 1000.0, 1_000_000.0)
bearings = (45.0, 135.0, 225.0, 315.0)

def test_line_of_bearing():
    for start_lon, start_lat in lon_lats:
        for distance in distances_meters:
            for bearing in bearings:
                dest_lon1, dest_lat1 = georgio.line_of_bearing(start_lon, start_lat, bearing, distance)
                dest_lat2, dest_lon2, _ = geopy.distance.distance(meters=distance).destination(
                    (start_lat, start_lon),
                    bearing=bearing,
                )
                assert dest_lon1 == pytest.approx(dest_lon2, abs=0.05)
                assert dest_lat1 == pytest.approx(dest_lat2, abs=0.05)

def test_line_of_bearing_benchmark():
    georgio_results = []

    for start_lon, start_lat in lon_lats:
        for distance in distances_meters:
            for bearing in bearings:
                run_time = timeit(
                    f'line_of_bearing({start_lon}, {start_lat}, {bearing}, {distance})',
                    setup='from georgio import line_of_bearing',
                    number=1000,
                )
                georgio_results.append(run_time/10_000*1_000_000_000)

    print(f'georgio fastest run: {min(georgio_results)} ns, slowest run: {max(georgio_results)} ns')

    geopy_results = []

    for start_lon, start_lat in lon_lats:
        for distance in distances_meters:
            for bearing in bearings:
                run_time = timeit(
                    f'distance(meters={distance}).destination(({start_lat}, {start_lon}), bearing={bearing})',
                    setup='from geopy.distance import distance',
                    number=1000,
                )
                geopy_results.append(run_time/10_000*1_000_000_000)

    print(f'geopy fastest run: {min(geopy_results)} ns, slowest run: {max(geopy_results)} ns')

    # makes sure georgio's slowest run is faster than geopy's fastest run
    assert max(georgio_results) < min(geopy_results)
