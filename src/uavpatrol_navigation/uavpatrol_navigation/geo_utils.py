"""Small WGS84/local ENU helpers for UAV patrol planning."""

import math


METERS_PER_DEGREE = 111194.9


def latlon_to_local(lat, lon, origin_lat, origin_lon, alt=0.0):
    """WGS84 latitude/longitude to local ENU meters."""
    origin_lat_rad = math.radians(origin_lat)
    x = (lon - origin_lon) * METERS_PER_DEGREE * math.cos(origin_lat_rad)
    y = (lat - origin_lat) * METERS_PER_DEGREE
    return x, y, alt


def local_to_latlon(x, y, origin_lat, origin_lon):
    """Local ENU meters to WGS84 latitude/longitude."""
    cos_phi = math.cos(math.radians(origin_lat))
    if abs(cos_phi) < 1e-9:
        raise ValueError("origin latitude is too close to the poles")
    lon = origin_lon + x / (METERS_PER_DEGREE * cos_phi)
    lat = origin_lat + y / METERS_PER_DEGREE
    return lat, lon
