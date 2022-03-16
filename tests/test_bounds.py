from timeit import timeit

import mercantile
import numba
import numpy as np
import pytest

import georgia

from .locations import lon_lats

@numba.njit(fastmath=True)
def jit_ul(xtile, ytile, zoom):
    inv_z2 = 1 / 2 ** zoom
    return (
        # lon_deg
        xtile * inv_z2 * 360.0 - 180.0,
        # lat_deg
        np.degrees(np.arctan(np.sinh(np.pi * (1 - 2 * ytile * inv_z2)))),
    )

@numba.njit(fastmath=True)
def jit_bounds(xtile, ytile, zoom, epsilon=1.0e-9):
    a = jit_ul(xtile, ytile, zoom)
    b = jit_ul(xtile + 1, ytile + 1, zoom)
    return a[0], b[1] + epsilon, b[0] - epsilon, a[1]  # west, south, east, north

def test_wm_upper_left():
    jit_lon, jit_lat = jit_ul(1205, 1540, 12)
    georgia_lon, georgia_lat = georgia.wm_upper_left(1205, 1540, 12)
    assert georgia_lon == pytest.approx(jit_lon)
    assert georgia_lat == pytest.approx(jit_lat)

def test_wm_upper_left_benchmark():
    georgia_results = []

    for lon_lat in lon_lats:
        for zoom in range(2, 20):
            tile = mercantile.tile(lon_lat[0], lon_lat[1], zoom)
            georgia_results.append(timeit(
                f'wm_upper_left({tile.x}, {tile.y}, {tile.z})',
                setup='from georgia import wm_upper_left',
                number=10_000,
            ))

    print(f'georgia fastest run: {min(georgia_results)}, slowest run: {max(georgia_results)}')

    jit_results = []

    for lon_lat in lon_lats:
        for zoom in range(2, 20):
            tile = mercantile.tile(lon_lat[0], lon_lat[1], zoom)
            jit_results.append(timeit(
                f'jit_ul({tile.x}, {tile.y}, {tile.z})',
                globals=globals(),
                number=10_000,
            ))

    print(f'jit fastest run: {min(jit_results)}, slowest run: {max(jit_results)}')

    # makes sure georgia's slowest run is faster than jit's fastest run
    #assert max(georgia_results) < min(jit_results)

def test_wm_bounds():
    j_west, j_south, j_east, j_north = jit_bounds(1205, 1540, 12)
    g_west, g_south, g_east, g_north = georgia.wm_bounds(1205, 1540, 12)
    assert g_west == pytest.approx(j_west)
    assert g_south == pytest.approx(j_south)
    assert g_east == pytest.approx(j_east)
    assert g_north == pytest.approx(j_north)

def test_wm_bounds_benchmark():
    georgia_results = []

    for lon_lat in lon_lats:
        for zoom in range(2, 20):
            tile = mercantile.tile(lon_lat[0], lon_lat[1], zoom)
            georgia_results.append(timeit(
                f'wm_bounds({tile.x}, {tile.y}, {tile.z})',
                setup='from georgia import wm_bounds',
                number=10_000,
            ))

    print(f'georgia fastest run: {min(georgia_results)}, slowest run: {max(georgia_results)}')

    jit_results = []

    for lon_lat in lon_lats:
        for zoom in range(2, 20):
            tile = mercantile.tile(lon_lat[0], lon_lat[1], zoom)
            jit_results.append(timeit(
                f'jit_bounds({tile.x}, {tile.y}, {tile.z})',
                globals=globals(),
                number=10_000,
            ))

    print(f'jit fastest run: {min(jit_results)}, slowest run: {max(jit_results)}')

    # makes sure georgia's slowest run is faster than jit's fastest run
    #assert max(georgia_results) < min(jit_results)
