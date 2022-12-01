# georgio

Fast **geo** **R**ust helper functions for Python.

## Great Circle

To get the great circle distance between two points in meters, this function will use the IUGG mean Earth radius (same as geopy).

```python
distance_in_meters = georgio.great_circle_distance(lon1, lat1, lon2, lat2)
```

If you want to provide your own radius in meters, you can use this function instead.

```python
distance_in_meters = georgio.great_circle_distance_with_radius(lon1, lat1, lon2, lat2, radius_in_meters)
```

## Bounding Box

This function will return a bounding box that's this specified distance around a particular center point.
Note that the bounding box will never extend across the antimeridian (longitude +/-180), below latitude -90, or above latitude 90.

```python
west, south, east, north = georgio.bounding_box_for_point(lon, lat, distance_in_meters)
```

## Web Mercator

To get the longitude/latitude bounds of a Web Mercator tile, use the following function, which will return the values in west, south, east, north order.

```python
west, south, east, north = georgio.wm_bounds(x, y, z)
```

To get the upper left corner of a tile in longitude, latitude order, use the following function.

```python
longitude, latitude = georgio.wm_upper_left(x, y, z)
```

To get a bounding box that surrounds a tile at a certain distance, use the following function.
This can be useful when searching for centerpoints of objects that might be outside of the tile, where the object might extend into the tile.
Note that the bounding box size is calculated in the WGS-84 projection based on the tile's longitude/latitude bounds, since Web Mercator is notoriously bad for calculating sizes/distances.
Also note that the bounding box will never extend across the antimeridian (longitude +/-180), below latitude -90, or above latitude 90.

```python
west, south, east, north = georgio.wm_tile_expanded_bbox(x, y, z, search_distance_in_meters)
```
