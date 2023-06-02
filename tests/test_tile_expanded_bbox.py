from timeit import timeit

import geopy.distance
import mercantile
import pytest

import georgio

from .locations import x_y_z_extensions

@pytest.mark.parametrize("x, y, z, extension_meters", x_y_z_extensions)
def test_wm_tile_expanded_bbox(x, y, z, extension_meters):
    exp_west, exp_south, exp_east, exp_north = georgio.wm_tile_expanded_bbox(x, y, z, extension_meters)
    tile_west, tile_south, tile_east, tile_north = mercantile.bounds(x, y, z)
    assert exp_west <= tile_west
    assert exp_south <= tile_south
    assert exp_east >= tile_east
    assert exp_north >= tile_north

    exp_center_lon = (exp_west + exp_east) / 2
    exp_center_lat = (exp_north + exp_south) / 2
    tile_center_lon = (tile_west + tile_east) / 2
    tile_center_lat = (tile_north + tile_south) / 2

    # Note that expanded bbox center will not
    # be the same as the tile center when near
    # the antimeridian or poles since the expanded
    # bbox cannot cross these boundaries.
    assert exp_center_lon == pytest.approx(tile_center_lon)
    assert exp_center_lat == pytest.approx(tile_center_lat)

    tile_corner_to_corner = geopy.distance.great_circle((tile_north, tile_west), (tile_south, tile_east)).m
    tile_extended_corner_to_corner = tile_corner_to_corner + (2 * extension_meters)
    bbox_corner_to_corner = geopy.distance.great_circle((exp_north, exp_west), (exp_south, exp_east)).m

    assert bbox_corner_to_corner >= tile_extended_corner_to_corner

    tile_corner_to_bbox_corner = geopy.distance.great_circle((tile_north, tile_east), (exp_north, exp_east)).m
    assert tile_corner_to_bbox_corner >= extension_meters

def test_wm_tile_expanded_bbox_nan():
    # This will raise an uncatchable pyo3_runtime.PanicException
    # if not handled properly in Rust.
    georgio.wm_tile_expanded_bbox(0, 0, 1, 1)

@pytest.mark.parametrize("x, y, z, extension_meters", x_y_z_extensions)
def test_wm_tile_expanded_bbox_benchmark(x, y, z, extension_meters):
    run_time = timeit(
        f"wm_tile_expanded_bbox({x}, {y}, {z}, {extension_meters})",
        setup="from georgio import wm_tile_expanded_bbox",
        number=100_000,
    )
    print("Average run time: {} ns".format(run_time/100_000*1_000_000_000))
