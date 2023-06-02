/*
 * File: lib.rs
 * Copyright (c) 2023, Spectric Labs Inc., All rights reserved.
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
use pyo3::prelude::*;

const EARTH_RADIUS_METERS: f32 = 6_371_008.8;  // IUGG mean Earth radius
const EPSILON32: f32 = 1.0e-9;

/// Returns true if the specified coordinates are valid.
fn valid_coordinates(lon: f32, lat: f32) -> bool {
    (-90.0..=90.0).contains(&lat) && (-180.0..=180.0).contains(&lon)
}

/// Returns the greate circle distance in meters for a sphere of the specified radius.
///
/// # Arguments
///
/// * `lon1` - The longitude of the start location.
/// * `lat1` - The latitude of the start location.
/// * `lon2` - The longitude of the end location.
/// * `lat2` - The latitude of the end location.
/// * `radius` - The radius of the sphere in meters.
#[pyfunction]
fn great_circle_distance_with_radius(lon1: f32, lat1: f32, lon2: f32, lat2: f32, radius: f32) -> PyResult<f32> {
    assert!(valid_coordinates(lon1, lat1));
    assert!(valid_coordinates(lon2, lat2));

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

/// Returns the great circle distance in meters for a sphere
/// with Earth's IUGG mean radius.
///
/// # Arguments
/// * `lon1` - The longitude of the start location.
/// * `lat1` - The latitude of the start location.
/// * `lon2` - The longitude of the end location.
/// * `lat2` - The latitude of the end location.
#[pyfunction]
fn great_circle_distance(lon1: f32, lat1: f32, lon2: f32, lat2: f32) -> PyResult<f32> {
    great_circle_distance_with_radius(lon1, lat1, lon2, lat2, EARTH_RADIUS_METERS)
}

/// Returns coordinates at the specified distance and bearing
/// for a sphere with the specified radius.
///
/// # Arguments
/// * `lon` - The longitude of the start point.
/// * `lat` - The latitude of the start point.
/// * `bearing` - The bearing to follow in degrees, clockwise from north.
/// * `distance` - The distance in meters.
/// * `radius` - The radius in meters.
#[pyfunction]
fn line_of_bearing_with_radius(lon: f32, lat: f32, bearing: f32, distance: f32, radius: f32) -> PyResult<(f32, f32)> {
    let rad_bearing = f32::to_radians(bearing);
    let rad_lat = f32::to_radians(lat);
    let rad_lon = f32::to_radians(lon);
    let d_div_r = distance / radius;
    let rad_new_lat = f32::asin(
        f32::sin(rad_lat)*f32::cos(d_div_r) +
        f32::cos(rad_lat)*f32::sin(d_div_r)*f32::cos(rad_bearing)
    );
    let rad_new_lon = rad_lon + f32::atan2(
        f32::sin(rad_bearing)*f32::sin(d_div_r)*f32::cos(rad_lat),
        f32::cos(d_div_r)-f32::sin(rad_lat)*f32::sin(rad_new_lat)
    );
    let new_lat = f32::to_degrees(rad_new_lat);
    let new_lon = f32::to_degrees(rad_new_lon);
    Ok((new_lon, new_lat))
}

/// Returns coordinates at the specified distance and bearing
/// for a sphere with Earth's IUGG mean radius.
///
/// # Arguments
/// * `lon` - The longitude of the start point.
/// * `lat` - The latitude of the start point.
/// * `bearing` - The bearing to follow in degrees, clockwise from north.
/// * `distance` - The distance in meters.
#[pyfunction]
fn line_of_bearing(lon: f32, lat: f32, bearing: f32, distance: f32) -> PyResult<(f32, f32)> {
    line_of_bearing_with_radius(lon, lat, bearing, distance, EARTH_RADIUS_METERS)
}

/// Returns a bounding box around a point
/// Returns the bounding box boundaries in the following order:
/// * West longitude
/// * South latitude
/// * East longitude
/// * North latitude
///
/// Note that the bounding box will never extend across the antimeridian
/// (longitude +/-180), below latitude -90, or above latitude 90.
///
/// # Arguments
/// * `lon` - The longitude of the starting point.
/// * `lat` - The latitude of the starting point.
/// * `distance_meters` - The distance to each side of the bounding box from the starting point.
#[pyfunction]
fn bounding_box_for_point(lon: f32, lat: f32, distance_meters: f32) -> PyResult<(f32, f32, f32, f32)> {
    assert!(valid_coordinates(lon, lat));
    assert!(distance_meters >= 0.0);

    let dist_radians = distance_meters / EARTH_RADIUS_METERS;
    let lon_rad = f32::to_radians(lon);
    let lat_rad = f32::to_radians(lat);
    let delta_lon_rad = f32::asin(f32::sin(dist_radians) / f32::cos(lat_rad));

    let north_rad = lat_rad + dist_radians;
    let south_rad = lat_rad - dist_radians;
    let east_rad = lon_rad + delta_lon_rad;
    let west_rad = lon_rad - delta_lon_rad;

    let bbox_north = f32::to_degrees(north_rad);
    let bbox_south = f32::to_degrees(south_rad);
    let bbox_east = f32::to_degrees(east_rad);
    let bbox_west = f32::to_degrees(west_rad);

    Ok((
        restrict_longitude(bbox_west),
        restrict_latitude(bbox_south),
        restrict_longitude(bbox_east),
        restrict_latitude(bbox_north),
    ))
}

/// Returns the upper-left longitude and latitude for a
/// Web Mercator tile.  This function is implemented separately
/// because it is used by the exposed `wm_upper_left` pyfunction,
/// and by `_wm_bounds`.
///
/// # Arguments
/// * x - Tile x coordinate
/// * y - Tile y coordinate
/// * z - Tile zoom level
fn _wm_upper_left(x: u32, y: u32, z: u32) -> (f32, f32) {
    // useful link: https://www.maptiler.com/google-maps-coordinates-tile-bounds-projection/
    let inv_z2: f32 = 1.0 / 2.0_f32.powi(z as i32);
    let lon = (x as f32) * inv_z2 * 360.0 - 180.0;
    let lat = f32::to_degrees(f32::atan(f32::sinh(PI * (1.0 - 2.0 * (y as f32) * inv_z2))));
    (lon, lat)
}

/// Returns the upper-left longitude and latitude for a
/// Web Mercator tile.
///
/// # Arguments
/// * x - Tile x coordinate
/// * y - Tile y coordinate
/// * z - Tile zoom level
#[pyfunction]
fn wm_upper_left(x: u32, y: u32, z:u32) -> PyResult<(f32, f32)> {
    Ok(_wm_upper_left(x, y, z))
}

/// Returns the boundaries for a Web Mercator tile in
/// the following order:
/// * West longitude
/// * South latitude
/// * East longitude
/// * North latitude
///
/// This function is implemented separately because it is
/// used by the exposed `wm_bounds` pyfunction as well as
/// `wm_tile_expanded_bbox`.
///
/// # Arguments
/// * x - Tile x coordinate
/// * y - Tile y coordinate
/// * z - Tile zoom level
fn _wm_bounds(x: u32, y: u32, z: u32) -> (f32, f32, f32, f32) {
    let a = _wm_upper_left(x, y, z);
    let b = _wm_upper_left(x+1, y+1, z);
    (a.0, b.1+EPSILON32, b.0-EPSILON32, a.1) // west, south, east, north
}

/// Returns the boundaries for a Web Mercator tile in
/// the following order:
/// * West longitude
/// * South latitude
/// * East longitude
/// * North latitude
///
/// # Arguments
/// * x - Tile x coordinate.
/// * y - Tile y coordinate.
/// * z - Tile zoom level.
#[pyfunction]
fn wm_bounds(x: u32, y: u32, z: u32) -> PyResult<(f32, f32, f32, f32)> {
    Ok(_wm_bounds(x, y, z))
}

/// Returns the center coordinates of a tile with the specified boundaries
/// as longitude and latitude.
///
/// # Arguments
/// * west - Western longitude boundary.
/// * south - Southern latitude boundary.
/// * east - Eastern longitude boundary.
/// * norht - Northern latitude boundary.
fn tile_center_lon_lat(west: f32, south: f32, east: f32, north: f32) -> (f32, f32) {
    ((west + east) / 2.0, (south + north) / 2.0)
}

/// Ensures a longitude between -180 and 180 (inclusive).
/// Longitudes beyond -180 are converted to -180.
/// Longitudes beyond 180 are converted to 180.
/// Longitudes in between -180 and 180 are returned
/// as they are.
///
/// # Arguments
/// * lon - The longitude to restrict.
fn restrict_longitude(lon: f32) -> f32 {
    if lon < -180.0 {
        -180.0
    } else if lon > 180.0 {
            180.0
    } else {
        lon
    }
}

/// Ensures a latitude between -90 and 90 (inclusive).
/// Latitudes beyond -90 are converted to -90.
/// Latitudes beyond 90 are converted to 90.
/// Latitudes in between -90 and 90 are returned
/// as they are.
///
/// # Arguments
/// * lat - The latitude to restrict.
fn restrict_latitude(lat: f32) -> f32 {
    if lat < -90.0 {
        -90.0
    } else if lat > 90.0 {
            90.0
    } else {
        lat
    }
}

/// Returns a bounding box that surrounds a tile at a certain distance.
/// Returns the bounding box boundaries in the following order:
/// * West longitude
/// * South latitude
/// * East longitude
/// * North latitude
///
/// This function can be useful when searching for centerpoints of objects
/// that might be outside of the tile, where the object might extend into
/// the tile.  Note that the bounding box size is calculated in the WGS-84
/// projection based on the tile's longitude/latitude bounds, since Web
/// Mercator is notoriously bad for calculating sizes/distances.  Also
/// note that the bounding box will never extend across the antimeridian
/// (longitude +/-180), below latitude -90, or above latitude 90.
///
/// # Arguments
/// * x - Tile x coordinate.
/// * y - Tile y coordinate.
/// * z - Tile zoom level.
/// * expansion_meters - Number of meters to expand the bounding box beyond the tile boundaries.
#[pyfunction]
fn wm_tile_expanded_bbox(x: u32, y: u32, z: u32, expansion_meters: f32) -> PyResult<(f32, f32, f32, f32)> {
    assert!(expansion_meters >= 0.0);
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

    // have to handle case where delta_lon_rad is NaN, like when x=0, y=0, z=1, expansion_meters=1
    let bbox_east = if !delta_lon_rad.is_nan() { f32::to_degrees(east_rad) } else { tile_east };
    let bbox_west = if !delta_lon_rad.is_nan() { f32::to_degrees(west_rad) } else { tile_west };

    Ok((
        restrict_longitude(bbox_west),
        restrict_latitude(bbox_south),
        restrict_longitude(bbox_east),
        restrict_latitude(bbox_north),
    ))
}

/// Specifies the functions to expose to Python.
#[pymodule]
fn georgio(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(great_circle_distance_with_radius, m)?)?;
    m.add_function(wrap_pyfunction!(great_circle_distance, m)?)?;
    m.add_function(wrap_pyfunction!(line_of_bearing_with_radius, m)?)?;
    m.add_function(wrap_pyfunction!(line_of_bearing, m)?)?;
    m.add_function(wrap_pyfunction!(bounding_box_for_point, m)?)?;
    m.add_function(wrap_pyfunction!(wm_bounds, m)?)?;
    m.add_function(wrap_pyfunction!(wm_upper_left, m)?)?;
    m.add_function(wrap_pyfunction!(wm_tile_expanded_bbox, m)?)?;
    Ok(())
}
