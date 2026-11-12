#include "trackConstraints.h"
#include <algorithm>

namespace mpc_controller {

TrackConstraints::TrackConstraints()
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

TrackConstraints::TrackConstraints(const PathToJson& path)
    : params_(Params())
{
}

TrackConstraints::TrackConstraints(const Params& params)
    : params_(params)
{
}

TrackConstraint TrackConstraints::getTrackConstraints(const ArcSpline& track, double s) const {
    // Get r_in and r_out from params_ member (loaded from JSON)
    double r_in  = params_.r_inner;
    double r_out = params_.r_outer;

    // X-Y point of the center line
    const Eigen::Vector2d pos_center = track.getPosition(s);
    const Eigen::Vector2d d_center   = track.getDerivative(s);

    // Tangent of center line at s (perpendicular to derivative)
        // Tangent of center line at s (perpendicular to derivative)
    Eigen::Vector2d tan_center = Eigen::Vector2d(-d_center(1), d_center(0));

    // Normalize to ensure boundaries are at exact r_in / r_out distance
    if (tan_center.norm() > 1e-6) {
        tan_center.normalize();
    } else {
        // Fallback to avoid zero-vector if derivative vanishes
        tan_center = Eigen::Vector2d(0.0, 1.0);
    }

    // Inner and outer track boundary given left and right width of track


    // Inner and outer track boundary given left and right width of track
    // TODO: make r_out and r_in dependent on s
    const Eigen::Vector2d pos_outer = pos_center + r_out * tan_center;
    const Eigen::Vector2d pos_inner = pos_center - r_in  * tan_center;

    // Define track Jacobian as perpendicular vector
    TrackConstraint track_constraint;
    track_constraint.C(0, 0) = tan_center(0);
    track_constraint.C(0, 1) = tan_center(1);

    // Compute bounds using min/max so the constraint remains correct even if
    // the normal direction flips at the closed-loop seam.
    const double bound_inner = tan_center.dot(pos_inner);
    const double bound_outer = tan_center.dot(pos_outer);
    track_constraint.lower = std::min(bound_inner, bound_outer);
    track_constraint.upper = std::max(bound_inner, bound_outer);

    return track_constraint;
}

} // namespace mpc_controller
