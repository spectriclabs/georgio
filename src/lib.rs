use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

const EARTH_RADIUS_METERS: f32 = 6_371_009.0;  // https://en.wikipedia.org/wiki/Earth_radius#Mean_radius
const EPSILON32: f32 = 1.0e-9;
const PI32: f32 = 3.141592653589793238462643383279502884;

fn valid_latitude(lat: f32) -> bool {
    lat > -90.0 && lat < 90.0
}

fn valid_longitude(lon: f32) -> bool {
    lon > -180.0 && lon < 180.0
}

fn valid_coordinates(lon: f32, lat: f32) -> bool {
    valid_longitude(lon) && valid_latitude(lat)
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
    let lat = f32::to_degrees(f32::atan(f32::sinh(PI32 * (1.0 - 2.0 * (y as f32) * inv_z2))));
    (lon, lat)
}

#[pyfunction]
fn wm_upper_left(x: u32, y: u32, z:u32) -> PyResult<(f32, f32)> {
    let lon_lat = _wm_upper_left(x, y, z);
    Ok((lon_lat.0, lon_lat.1))
}

#[pyfunction]
fn wm_bounds(x: u32, y: u32, z: u32) -> PyResult<(f32, f32, f32, f32)> {
    let a = _wm_upper_left(x, y, z);
    let b = _wm_upper_left(x+1, y+1, z);
    Ok((a.0, b.1+EPSILON32, b.0-EPSILON32, a.1)) // west, south, east, north
}

#[pymodule]
fn georgia(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(great_circle_distance_with_radius, m)?)?;
    m.add_function(wrap_pyfunction!(great_circle_distance, m)?)?;
    m.add_function(wrap_pyfunction!(wm_bounds, m)?)?;
    m.add_function(wrap_pyfunction!(wm_upper_left, m)?)?;
    Ok(())
}
