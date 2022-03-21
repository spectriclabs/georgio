/*
 * File: lib.rs
 * Copyright (c) 2022, Spectric Labs Inc., All rights reserved.
 *
 * This file is part of the georgio Python package.
 *
 * Licensed to the Spectric Labs (Spectric) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  Spectric licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

use std::f32::consts::PI;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

const EARTH_RADIUS_METERS: f32 = 6_371_009.0;  // https://en.wikipedia.org/wiki/Earth_radius#Mean_radius
const EPSILON32: f32 = 1.0e-9;

fn valid_coordinates(lon: f32, lat: f32) -> bool {
    lat > -90.0 && lat < 90.0 && lon > -180.0 && lon < 180.0
}

#[pyfunction]
fn great_circle_distance_with_radius(lon1: f32, lat1: f32, lon2: f32, lat2: f32, radius: f32) -> PyResult<f32> {
    if !valid_coordinates(lon1, lat1) || !valid_coordinates(lon2, lat2) {
        return Err(PyValueError::new_err("Invalid coordinates"));
    }

    // see https://github.com/geopy/geopy/blob/master/geopy/distance.py
    let lat1_rad = lat1.to_radians();
    let lon1_rad = lon1.to_radians();

    let lat2_rad = lat2.to_radians();
    let lon2_rad = lon2.to_radians();

    let sin_lat1 = lat1_rad.sin();
    let cos_lat1 = lat1_rad.cos();

    let sin_lat2 = lat2_rad.sin();
    let cos_lat2 = lat2_rad.cos();

    let delta_lon_rad = lon2_rad - lon1_rad;
    let cos_delta_lon = delta_lon_rad.cos();
    let sin_delta_lon = delta_lon_rad.sin();

    let d = f32::atan2(f32::sqrt((cos_lat2 * sin_delta_lon).powi(2) +
             (cos_lat1 * sin_lat2 -
              sin_lat1 * cos_lat2 * cos_delta_lon).powi(2)),
              sin_lat1 * sin_lat2 + cos_lat1 * cos_lat2 * cos_delta_lon);
    Ok(radius * d)
}

#[pyfunction]
fn great_circle_distance(lon1: f32, lat1: f32, lon2: f32, lat2: f32) -> PyResult<f32> {
    great_circle_distance_with_radius(lon1, lat1, lon2, lat2, EARTH_RADIUS_METERS)
}

fn _wm_upper_left(x: u32, y: u32, z: u32) -> (f32, f32) {
    let inv_z2: f32 = 1.0 / 2.0_f32.powi(z as i32);
    let lon = (x as f32) * inv_z2 * 360.0 - 180.0;
    let lat = f32::to_degrees(f32::atan(f32::sinh(PI * (1.0 - 2.0 * (y as f32) * inv_z2))));
    (lon, lat)
}

#[pyfunction]
fn wm_upper_left(x: u32, y: u32, z:u32) -> PyResult<(f32, f32)> {
    let lon_lat = _wm_upper_left(x, y, z);
    Ok((lon_lat.0, lon_lat.1))
}

fn _wm_bounds(x: u32, y: u32, z: u32) -> (f32, f32, f32, f32) {
    let a = _wm_upper_left(x, y, z);
    let b = _wm_upper_left(x+1, y+1, z);
    (a.0, b.1+EPSILON32, b.0-EPSILON32, a.1) // west, south, east, north
}

#[pyfunction]
fn wm_bounds(x: u32, y: u32, z: u32) -> PyResult<(f32, f32, f32, f32)> {
    Ok(_wm_bounds(x, y, z))
}

fn tile_center_lon_lat(west: f32, south: f32, east: f32, north: f32) -> (f32, f32) {
    ((west + east) / 2.0, (south + north) / 2.0)
}

#[pyfunction]
fn wm_tile_expanded_bbox(x: u32, y: u32, z: u32, expansion_meters: f32) -> PyResult<(f32, f32, f32, f32)> {
    // based on http://janmatuschek.de/LatitudeLongitudeBoundingCoordinates
    let (tile_west, tile_south, tile_east, tile_north) = _wm_bounds(x, y, z);

    // want a search circle that encompasses the tile, not a tile that encompasses the search circle
    let corner_to_corner_meters = great_circle_distance(tile_west, tile_north, tile_east, tile_south)?;
    let search_meters = (corner_to_corner_meters / 2.0) + expansion_meters;
    let search_radians = search_meters / EARTH_RADIUS_METERS;

    let (tile_center_lon, tile_center_lat) = tile_center_lon_lat(tile_west, tile_south, tile_east, tile_north);
    let tile_center_lon_rad = f32::to_radians(tile_center_lon);
    let tile_center_lat_rad = f32::to_radians(tile_center_lat);
    let delta_lon_rad = f32::asin(f32::sin(search_radians) / f32::cos(tile_center_lat_rad));

    let north_rad = tile_center_lat_rad + search_radians;
    let south_rad = tile_center_lat_rad - search_radians;
    let east_rad = tile_center_lon_rad + delta_lon_rad;
    let west_rad = tile_center_lon_rad - delta_lon_rad;

    let bbox_north = f32::to_degrees(north_rad);
    let bbox_south = f32::to_degrees(south_rad);
    let bbox_east = f32::to_degrees(east_rad);
    let bbox_west = f32::to_degrees(west_rad);

    Ok((bbox_west, bbox_south, bbox_east, bbox_north))
}

#[pymodule]
fn georgio(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(great_circle_distance_with_radius, m)?)?;
    m.add_function(wrap_pyfunction!(great_circle_distance, m)?)?;
    m.add_function(wrap_pyfunction!(wm_bounds, m)?)?;
    m.add_function(wrap_pyfunction!(wm_upper_left, m)?)?;
    m.add_function(wrap_pyfunction!(wm_tile_expanded_bbox, m)?)?;
    Ok(())
}
