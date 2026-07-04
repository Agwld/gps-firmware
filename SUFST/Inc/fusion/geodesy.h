/**
 * @file    geodesy.h
 * @brief   WGS84 lat/lon/height to local East-North-Up (ENU) tangent-plane
 *          conversion, relative to an origin fix set once at startup.
 *
 * A race circuit spans at most a few km, so a full ECEF round-trip with
 * an ENU rotation matrix is unnecessary precision for no practical gain;
 * a flat-Earth local-tangent-plane (LTP) approximation is standard and
 * sufficient here. What does matter for lap-timing gates (sub-metre
 * crossing accuracy) is not approximating the *Earth* as a sphere: the
 * meters-per-degree scale factors are derived from the WGS84 ellipsoid's
 * actual radii of curvature at the origin latitude, not a spherical
 * approximation, so the East/North scaling stays accurate close to the
 * origin.
 */

#ifndef GEODESY_H
#define GEODESY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set the local ENU tangent-plane origin (e.g. the first GPS fix).
 *
 * Latitude/longitude are taken as double: a float32 degree value only
 * carries ~1 m of precision at these magnitudes, which is not enough
 * headroom for sub-metre gate timing once combined with the ENU maths
 * below, so the origin and every conversion input use double degrees.
 *
 * @param lat_deg    origin latitude, degrees.
 * @param lon_deg    origin longitude, degrees.
 * @param height_m   origin height, metres (any consistent height frame -
 *                   ellipsoidal or MSL - as long as later calls use the
 *                   same frame).
 */
void geodesy_set_origin(double lat_deg, double lon_deg, float height_m);

/**
 * @brief Convert a WGS84 lat/lon/height fix to ENU metres relative to the
 *        origin set by geodesy_set_origin().
 *
 * @param lat_deg, lon_deg, height_m  fix to convert (same conventions as
 *                                    geodesy_set_origin()).
 * @param east_m, north_m, up_m       [out] ENU offset from the origin, m.
 */
void geodesy_to_enu(double lat_deg, double lon_deg, float height_m,
                     float *east_m, float *north_m, float *up_m);

/**
 * @brief Inverse of geodesy_to_enu(): convert an ENU offset from the
 *        origin (e.g. the fused KF position) back to WGS84 lat/lon/
 *        height, using the same per-origin scale factors so it is exact
 *        to within the same flat-Earth approximation as the forward
 *        conversion (not an independent/higher-precision recomputation).
 *
 * @param east_m, north_m, up_m  ENU offset from the origin, m.
 * @param lat_deg, lon_deg       [out] degrees.
 * @param height_m               [out] same height frame as the origin.
 */
void geodesy_from_enu(float east_m, float north_m, float up_m,
                       double *lat_deg, double *lon_deg, float *height_m);

#ifdef __cplusplus
}
#endif

#endif /* GEODESY_H */
