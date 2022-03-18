from timeit import timeit

import geopy.distance
import mercantile
import pytest

import georgio

from .locations import x_y_z_extensions

@pytest.mark.parametrize("x, y, z, extension_meters", x_y_z_extensions)
def test_wm_tile_expanded_bbox(x, y, z, extension_meters):
    west, south, east, north = georgio.wm_tile_expanded_bbox(x, y, z, extension_meters)
    tile_west, tile_south, tile_east, tile_north = mercantile.bounds(x, y, z)
    assert west < tile_west
    assert south < tile_south
    assert east > tile_east
    assert north > tile_north

    center_lon = (west + east) / 2
    center_lat = (north + south) / 2
    tile_center_lon = (tile_west + tile_east) / 2
    tile_center_lat = (tile_north + tile_south) / 2

    assert center_lon == pytest.approx(tile_center_lon)
    assert center_lat == pytest.approx(tile_center_lat)

    tile_corner_to_corner = geopy.distance.great_circle((tile_north, tile_west), (tile_south, tile_east)).m
    tile_extended_corner_to_corner = tile_corner_to_corner + (2 * extension_meters)
    bbox_corner_to_corner = geopy.distance.great_circle((north, west), (south, east)).m

    assert bbox_corner_to_corner >= tile_extended_corner_to_corner

@pytest.mark.parametrize("x, y, z, extension_meters", x_y_z_extensions)
def test_wm_tile_expanded_bbox_benchmark(x, y, z, extension_meters):
    run_time = timeit(
        f"wm_tile_expanded_bbox({x}, {y}, {z}, {extension_meters})",
        setup="from georgio import wm_tile_expanded_bbox",
        number=100_000,
    )
    print("Average run time: {} ns".format(run_time/100_000*1_000_000_000))
