from itertools import permutations
from timeit import timeit

import pytest

import geopy.distance
import georgio

from .locations import lon_lats

def test_great_circle_distance():
    for pair in permutations(lon_lats, 2):
        lon1 = pair[0][0]
        lat1 = pair[0][1]
        lon2 = pair[1][0]
        lat2 = pair[1][1]
        dist1 = georgio.great_circle_distance_with_radius(lon1, lat1, lon2, lat2, geopy.distance.EARTH_RADIUS*1000)
        dist2 = geopy.distance.great_circle((lat1, lon1), (lat2, lon2)).m
        assert dist1 == pytest.approx(dist2)

def test_great_circle_benchmark():
    georgio_results = []

    for pair in permutations(lon_lats, 2):
        lon1 = pair[0][0]
        lat1 = pair[0][1]
        lon2 = pair[1][0]
        lat2 = pair[1][1]
        georgio_results.append(timeit(
            f'great_circle_distance({lon1}, {lat1}, {lon2}, {lat2})',
            setup='from georgio import great_circle_distance',
            number=10_000,
        ))

    print(f'georgio fastest run: {min(georgio_results)}, slowest run: {max(georgio_results)}')

    geopy_results = []

    for pair in permutations(lon_lats, 2):
        lon1 = pair[0][0]
        lat1 = pair[0][1]
        lon2 = pair[1][0]
        lat2 = pair[1][1]
        geopy_results.append(timeit(
            f'great_circle(({lat1}, {lon1}), ({lat2}, {lon2})).m',
            setup='from geopy.distance import great_circle',
            number=10_000,
        ))

    print(f'geopy fastest run: {min(geopy_results)}, slowest run: {max(geopy_results)}')

    # makes sure georgio's slowest run is faster than geopy's fastest run
    assert max(georgio_results) < min(geopy_results)
