/**
 * @file    geodesy.c
 * @brief   WGS84 -> local ENU flat-Earth tangent-plane conversion.
 */

#include "fusion/geodesy.h"

#include <math.h>

/* WGS84 ellipsoid constants (avoid relying on math.h's M_PI, which is a
 * glibc extension outside strict C11). */
#define GEODESY_PI 3.14159265358979323846
#define GEODESY_DEG_TO_RAD (GEODESY_PI / 180.0)

#define WGS84_A  6378137.0             /* semi-major axis, m */
#define WGS84_F  (1.0 / 298.257223563) /* flattening */
#define WGS84_E2 (WGS84_F * (2.0 - WGS84_F)) /* first eccentricity^2 */

static double s_origin_lat_deg;
static double s_origin_lon_deg;
static float s_origin_height_m;

/* Scale factors converting a degree of latitude/longitude at the origin
 * into metres, derived from the WGS84 meridian and normal radii of
 * curvature (not a spherical-Earth mean radius) so that the flat-Earth
 * approximation stays accurate to well under a metre across a circuit's
 * few-km extent. */
static float s_m_per_deg_lat;
static float s_m_per_deg_lon;

void
geodesy_set_origin(double lat_deg, double lon_deg, float height_m)
{
    s_origin_lat_deg = lat_deg;
    s_origin_lon_deg = lon_deg;
    s_origin_height_m = height_m;

    double lat_rad = lat_deg * GEODESY_DEG_TO_RAD;
    double sin_lat = sin(lat_rad);
    double denom = 1.0 - WGS84_E2 * sin_lat * sin_lat;
    double sqrt_denom = sqrt(denom);

    /* Meridian radius of curvature M (north-south) and normal radius of
     * curvature N (east-west); see any WGS84 geodesy reference. */
    double m_radius = (WGS84_A * (1.0 - WGS84_E2)) / (denom * sqrt_denom);
    double n_radius = WGS84_A / sqrt_denom;

    s_m_per_deg_lat = (float) (m_radius * GEODESY_DEG_TO_RAD);
    s_m_per_deg_lon = (float) (n_radius * cos(lat_rad) * GEODESY_DEG_TO_RAD);
}

void
geodesy_to_enu(double lat_deg, double lon_deg, float height_m,
               float *east_m, float *north_m, float *up_m)
{
    /* The degree deltas are computed in double before scaling to metres
     * in float: a float32 absolute latitude only carries ~1 m of
     * precision, but a float32 *delta* of a few km is fine, so the
     * precision-sensitive subtraction has to happen in double first. */
    double dlat_deg = lat_deg - s_origin_lat_deg;
    double dlon_deg = lon_deg - s_origin_lon_deg;

    *east_m = (float) (dlon_deg * (double) s_m_per_deg_lon);
    *north_m = (float) (dlat_deg * (double) s_m_per_deg_lat);
    *up_m = height_m - s_origin_height_m;
}

void
geodesy_from_enu(float east_m, float north_m, float up_m, double *lat_deg,
                  double *lon_deg, float *height_m)
{
    /* The scale factors are zero until geodesy_set_origin() runs on the
     * first fix. Dividing by them before then yields NaN/Inf; report the
     * (as-yet-unset) origin instead so callers never publish garbage. */
    if (s_m_per_deg_lat == 0.0f || s_m_per_deg_lon == 0.0f) {
        *lat_deg = s_origin_lat_deg;
        *lon_deg = s_origin_lon_deg;
        *height_m = s_origin_height_m + up_m;
        return;
    }

    *lat_deg = s_origin_lat_deg + (double) north_m / (double) s_m_per_deg_lat;
    *lon_deg = s_origin_lon_deg + (double) east_m / (double) s_m_per_deg_lon;
    *height_m = s_origin_height_m + up_m;
}
