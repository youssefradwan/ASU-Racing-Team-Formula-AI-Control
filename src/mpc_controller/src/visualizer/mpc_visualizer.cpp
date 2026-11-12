#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "mpc_visualizer.h"
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>

// ─────────────────────────────────────────────────────────────────────────────
static json loadJson(const std::string& path, rclcpp::Logger log)
{
    std::ifstream f(path);
    if (!f.is_open()) {
        RCLCPP_WARN(log, "Cannot open JSON file: %s", path.c_str());
        return json{};
    }
    try {
        return json::parse(f);
    } catch (const json::exception& e) {
        RCLCPP_ERROR(log, "JSON parse error in %s: %s", path.c_str(), e.what());
        return json{};
    }
}

// ─────────────────────────────────────────────────────────────────────────────
MPCVisualizer::MPCVisualizer()
    : Node("mpc_visualizer")
{
    /* ---------- parameters ---------- */
    declare_parameter("nodes_path",   std::string("params/nodes.json"));
    declare_parameter("car_length",   car_length_);
    declare_parameter("car_width",    car_width_);
    declare_parameter("wheelbase",    wheelbase_);
    declare_parameter("track_width",  track_width_);
    declare_parameter("r_inner",      r_inner_);
    declare_parameter("r_outer",      r_outer_);
    declare_parameter("cone_spacing", cone_spacing_);
    car_length_   = get_parameter("car_length").as_double();
    car_width_    = get_parameter("car_width").as_double();
    wheelbase_    = get_parameter("wheelbase").as_double();
    track_width_  = get_parameter("track_width").as_double();
    r_inner_      = get_parameter("r_inner").as_double();
    r_outer_      = get_parameter("r_outer").as_double();
    cone_spacing_ = get_parameter("cone_spacing").as_int();

    /* ---------- load topic names from nodes.json ---------- */
    const std::string nodes_path = get_parameter("nodes_path").as_string();
    const json nodes_cfg = loadJson(nodes_path, get_logger());
    const json topics    = nodes_cfg.value("topics", json{});

    const std::string t_odom     = topics.value("odometry",           "/carmaker/Odometry");
    const std::string t_path     = topics.value("reference_path",     "/path");
    const std::string t_pred     = topics.value("predicted_path",     "/mpc/predicted_path");
    const std::string t_markers  = topics.value("track_markers",      "/mpc/track_markers");
    const std::string t_heading  = topics.value("heading_arrow",      "/mpc/heading_arrow");
    const std::string t_foot     = topics.value("vehicle_footprint",  "/mpc/vehicle_footprint");
    const std::string t_constr   = topics.value("constraint_markers", "/mpc/constraint_markers");
    const std::string t_pred_viz = topics.value("predicted_path_viz", "/mpc/predicted_path_viz");

    RCLCPP_INFO(get_logger(), "nodes.json : %s", nodes_path.c_str());
    RCLCPP_INFO(get_logger(), "Topics     : odom=%s  path=%s  pred=%s",
        t_odom.c_str(), t_path.c_str(), t_pred.c_str());

    /* ---------- subscribers ---------- */
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        t_odom, 10,
        std::bind(&MPCVisualizer::odomCallback, this, std::placeholders::_1));
    ref_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        t_path, rclcpp::QoS(10).transient_local(),
        std::bind(&MPCVisualizer::pathCallback, this, std::placeholders::_1));
    pred_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        t_pred, 10,
        std::bind(&MPCVisualizer::predictedPathCallback, this, std::placeholders::_1));

    /* ---------- publishers ---------- */
    marker_pub_     = create_publisher<visualization_msgs::msg::MarkerArray>(
        t_markers, 10);
    heading_pub_    = create_publisher<visualization_msgs::msg::Marker>(
        t_heading, 10);
    footprint_pub_  = create_publisher<visualization_msgs::msg::Marker>(
        t_foot, 10);
    constraint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        t_constr, 10);
    pred_viz_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>(
        t_pred_viz, 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /* 20 Hz visualisation timer */
    vis_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MPCVisualizer::timerCallback, this));

    RCLCPP_INFO(get_logger(), "MPC Visualizer (CarMaker) started  car=%.1f×%.1fm  track_hw=%.1fm",
                car_length_, car_width_, track_width_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
double MPCVisualizer::getYawFromQuat(const geometry_msgs::msg::Quaternion& q) const
{
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double r, p, y;
    tf2::Matrix3x3(tq).getRPY(r, p, y);
    return y;
}

size_t MPCVisualizer::findClosestIndex(double px, double py) const
{
    double best_dist = 1e20;
    size_t best = 0;
    for (size_t j = 0; j < reference_path_->poses.size(); ++j) {
        double dx = reference_path_->poses[j].pose.position.x - px;
        double dy = reference_path_->poses[j].pose.position.y - py;
        double d  = dx * dx + dy * dy;
        if (d < best_dist) { best_dist = d; best = j; }
    }
    return best;
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void MPCVisualizer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_odom_ = msg;
    has_odom_    = true;
}

void MPCVisualizer::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    reference_path_ = msg;
    has_ref_path_   = true;
    fitTrackSpline();
    publishTrackMarkers();
}

void MPCVisualizer::predictedPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    predicted_path_ = msg;
    has_pred_path_  = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Fit ArcSpline + TrackConstraints from the received path
// ─────────────────────────────────────────────────────────────────────────────
void MPCVisualizer::fitTrackSpline()
{
    if (!has_ref_path_ || reference_path_->poses.empty()) return;
    const size_t n = reference_path_->poses.size();
    Eigen::VectorXd X(n), Y(n);
    for (size_t i = 0; i < n; ++i) {
        X(i) = reference_path_->poses[i].pose.position.x;
        Y(i) = reference_path_->poses[i].pose.position.y;
    }
    track_spline_.generateSpline(X, Y);

    // Build TrackConstraints using the current r_inner / r_outer ROS parameters
    mpc_controller::Params p;
    p.r_inner = r_inner_;
    p.r_outer = r_outer_;
    track_constraints_ = mpc_controller::TrackConstraints(p);

    has_spline_ = true;
    RCLCPP_INFO(get_logger(), "Track spline fitted: total_length=%.1f m",
                track_spline_.getTotalLength());
}

void MPCVisualizer::timerCallback()
{
    if (has_odom_) {
        publishHeadingArrow(*latest_odom_);
        publishVehicleFootprint(*latest_odom_);
        broadcastTF(*latest_odom_);
    }
    // Republish track markers at ~1 Hz so late-joining RViz always sees them
    if (has_ref_path_) {
        static int track_counter = 0;
        if (++track_counter >= 20) {  // 20 × 50 ms = 1 s
            publishTrackMarkers();
            track_counter = 0;
        }
    }
    if (has_pred_path_) {
        publishPredictedPath();
        publishTrackConstraints();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Track centerline (from fitted ArcSpline) + boundaries (from TrackConstraints)
// ─────────────────────────────────────────────────────────────────────────────
void MPCVisualizer::publishTrackMarkers()
{
    if (!has_spline_) return;

    visualization_msgs::msg::MarkerArray ma;
    auto stamp = now();

    // ━━ Centerline (LINE_STRIP, bright green) ━━━━━━━━━━━━━━━━━━━━━━━━━━
    visualization_msgs::msg::Marker center;
    center.header.frame_id = "map";
    center.header.stamp    = stamp;
    center.ns       = "track_center";
    center.id       = 0;
    center.type     = visualization_msgs::msg::Marker::LINE_STRIP;
    center.action   = visualization_msgs::msg::Marker::ADD;
    center.scale.x  = 0.06;
    center.color.r  = 0.2f; center.color.g = 0.9f; center.color.b = 0.2f; center.color.a = 1.0f;
    center.pose.orientation.w = 1.0;

    // ━━ Left boundary (LINE_STRIP, red — outer wall) ━━━━━━━━━━━━━━━━━━━
    visualization_msgs::msg::Marker left_wall = center;
    left_wall.ns = "track_left";  left_wall.id = 1;
    left_wall.scale.x = 0.04;
    left_wall.color.r = 1.0f; left_wall.color.g = 0.15f; left_wall.color.b = 0.15f; left_wall.color.a = 1.0f;

    // ━━ Right boundary (LINE_STRIP, blue — inner wall) ━━━━━━━━━━━━━━━━━
    visualization_msgs::msg::Marker right_wall = center;
    right_wall.ns = "track_right"; right_wall.id = 2;
    right_wall.scale.x = 0.04;
    right_wall.color.r = 0.15f; right_wall.color.g = 0.3f; right_wall.color.b = 1.0f; right_wall.color.a = 1.0f;

    // ━━ Left cones (SPHERE_LIST, orange — outer) ━━━━━━━━━━━━━━━━━━━━━━━
    visualization_msgs::msg::Marker left_cones;
    left_cones.header.frame_id = "map";
    left_cones.header.stamp    = stamp;
    left_cones.ns      = "cones_left";
    left_cones.id      = 3;
    left_cones.type    = visualization_msgs::msg::Marker::SPHERE_LIST;
    left_cones.action  = visualization_msgs::msg::Marker::ADD;
    left_cones.scale.x = 0.35; left_cones.scale.y = 0.35; left_cones.scale.z = 0.5;
    left_cones.color.r = 1.0f; left_cones.color.g = 0.4f; left_cones.color.b = 0.0f; left_cones.color.a = 1.0f;
    left_cones.pose.orientation.w = 1.0;

    // ━━ Right cones (SPHERE_LIST, blue — inner) ━━━━━━━━━━━━━━━━━━━━━━━━
    visualization_msgs::msg::Marker right_cones = left_cones;
    right_cones.ns = "cones_right"; right_cones.id = 4;
    right_cones.color.r = 0.0f; right_cones.color.g = 0.3f; right_cones.color.b = 1.0f;

    static constexpr int N_VIZ = 500;
    const double total_s = track_spline_.getTotalLength();

    // Compute track centroid from spline samples so that at the hairpin
    // (where the travel-direction normal flips 180°) the outer wall (further
    // from centroid) and inner wall (closer) are assigned consistently.
    double track_cx = 0.0, track_cy = 0.0;
    for (int i = 0; i < N_VIZ; ++i) {
        Eigen::Vector2d pos = track_spline_.getPosition(total_s * i / N_VIZ);
        track_cx += pos(0); track_cy += pos(1);
    }
    track_cx /= N_VIZ; track_cy /= N_VIZ;

    for (int i = 0; i < N_VIZ; ++i) {
        const double s = total_s * i / N_VIZ;

        // Smooth centerline point from ArcSpline
        Eigen::Vector2d pos_c = track_spline_.getPosition(s);
        geometry_msgs::msg::Point pc;
        pc.x = pos_c(0); pc.y = pos_c(1); pc.z = 0.0;
        center.points.push_back(pc);

        // TrackConstraints supplies the correct left-of-travel normal at s
        mpc_controller::TrackConstraint tc =
            track_constraints_.getTrackConstraints(track_spline_, s);
        const double nx = tc.C(0, 0);
        const double ny = tc.C(0, 1);

        // Candidate boundary points from TrackConstraints geometry
        geometry_msgs::msg::Point pa, pb;
        pa.x = pos_c(0) + r_outer_ * nx;  pa.y = pos_c(1) + r_outer_ * ny;  pa.z = 0.0;
        pb.x = pos_c(0) - r_inner_ * nx;  pb.y = pos_c(1) - r_inner_ * ny;  pb.z = 0.0;

        // Centroid-distance test: outer = further from centroid
        const double d_a = std::hypot(pa.x - track_cx, pa.y - track_cy);
        const double d_b = std::hypot(pb.x - track_cx, pb.y - track_cy);
        const geometry_msgs::msg::Point& p_outer = (d_a >= d_b) ? pa : pb;
        const geometry_msgs::msg::Point& p_inner = (d_a >= d_b) ? pb : pa;

        left_wall.points.push_back(p_outer);
        right_wall.points.push_back(p_inner);

        if (i % cone_spacing_ == 0) {
            geometry_msgs::msg::Point cl = p_outer, cr = p_inner;
            cl.z = 0.15; cr.z = 0.15;
            left_cones.points.push_back(cl);
            right_cones.points.push_back(cr);
        }
    }

    ma.markers.push_back(center);
    ma.markers.push_back(left_wall);
    ma.markers.push_back(right_wall);
    ma.markers.push_back(left_cones);
    ma.markers.push_back(right_cones);
    marker_pub_->publish(ma);
}

// ─────────────────────────────────────────────────────────────────────────────
// Heading + velocity arrow
// ─────────────────────────────────────────────────────────────────────────────
void MPCVisualizer::publishHeadingArrow(const nav_msgs::msg::Odometry& odom)
{
    double yaw = getYawFromQuat(odom.pose.pose.orientation);
    double v   = odom.twist.twist.linear.x;
    double arrow_len = std::max(1.0, std::min(v * 0.5, 4.0));  // scale with speed

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp    = now();
    arrow.ns   = "heading";
    arrow.id   = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = odom.pose.pose.position.x;
    start.y = odom.pose.pose.position.y;
    start.z = 0.2;
    end.x = start.x + arrow_len * std::cos(yaw);
    end.y = start.y + arrow_len * std::sin(yaw);
    end.z = 0.2;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    arrow.scale.x = 0.10;  // shaft diameter
    arrow.scale.y = 0.20;  // head diameter
    arrow.scale.z = 0.25;  // head length
    arrow.color.r = 1.0f; arrow.color.g = 0.9f; arrow.color.b = 0.0f; arrow.color.a = 1.0f;

    heading_pub_->publish(arrow);
}

// ─────────────────────────────────────────────────────────────────────────────
// Vehicle footprint — wireframe rectangle with 4 wheels (LINE_LIST in base_link)
// ─────────────────────────────────────────────────────────────────────────────
void MPCVisualizer::publishVehicleFootprint(const nav_msgs::msg::Odometry& /*odom*/)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp    = now();
    m.ns   = "vehicle_body";
    m.id   = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.04;  // line width
    m.color.r = 0.0f; m.color.g = 0.85f; m.color.b = 1.0f; m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;

    // Body corners (rear axle at origin, CG between axles)
    double rear_overhang  = (car_length_ - wheelbase_) * 0.35;
    double front_overhang = car_length_ - wheelbase_ - rear_overhang;
    double x_rear  = -rear_overhang;
    double x_front = wheelbase_ + front_overhang;
    double hw = car_width_ / 2.0;

    // Helper lambda
    auto pt = [](double x, double y, double z) {
        geometry_msgs::msg::Point p;
        p.x = x; p.y = y; p.z = z;
        return p;
    };
    double bz = 0.15;  // body z height

    // 4 edges of the body rectangle
    m.points.push_back(pt(x_rear,  -hw, bz)); m.points.push_back(pt(x_front, -hw, bz));
    m.points.push_back(pt(x_front, -hw, bz)); m.points.push_back(pt(x_front,  hw, bz));
    m.points.push_back(pt(x_front,  hw, bz)); m.points.push_back(pt(x_rear,   hw, bz));
    m.points.push_back(pt(x_rear,   hw, bz)); m.points.push_back(pt(x_rear,  -hw, bz));

    // Center line (rear axle → front axle)
    m.points.push_back(pt(0.0, 0.0, bz)); m.points.push_back(pt(wheelbase_, 0.0, bz));

    // Wheel markers: small rectangles at each axle corner
    double wl = 0.30;  // wheel length
    double ww = 0.06;  // wheel width offset
    // Rear-left
    m.points.push_back(pt(-wl/2, -hw - ww, bz)); m.points.push_back(pt(wl/2, -hw - ww, bz));
    m.points.push_back(pt(-wl/2,  -hw + ww, bz)); m.points.push_back(pt(wl/2, -hw + ww, bz));
    m.points.push_back(pt(-wl/2, -hw - ww, bz)); m.points.push_back(pt(-wl/2, -hw + ww, bz));
    m.points.push_back(pt( wl/2, -hw - ww, bz)); m.points.push_back(pt( wl/2, -hw + ww, bz));
    // Rear-right
    m.points.push_back(pt(-wl/2,  hw - ww, bz)); m.points.push_back(pt(wl/2,  hw - ww, bz));
    m.points.push_back(pt(-wl/2,  hw + ww, bz)); m.points.push_back(pt(wl/2,  hw + ww, bz));
    m.points.push_back(pt(-wl/2,  hw - ww, bz)); m.points.push_back(pt(-wl/2, hw + ww, bz));
    m.points.push_back(pt( wl/2,  hw - ww, bz)); m.points.push_back(pt( wl/2, hw + ww, bz));
    // Front-left
    double fx = wheelbase_;
    m.points.push_back(pt(fx-wl/2, -hw - ww, bz)); m.points.push_back(pt(fx+wl/2, -hw - ww, bz));
    m.points.push_back(pt(fx-wl/2, -hw + ww, bz)); m.points.push_back(pt(fx+wl/2, -hw + ww, bz));
    m.points.push_back(pt(fx-wl/2, -hw - ww, bz)); m.points.push_back(pt(fx-wl/2, -hw + ww, bz));
    m.points.push_back(pt(fx+wl/2, -hw - ww, bz)); m.points.push_back(pt(fx+wl/2, -hw + ww, bz));
    // Front-right
    m.points.push_back(pt(fx-wl/2,  hw - ww, bz)); m.points.push_back(pt(fx+wl/2,  hw - ww, bz));
    m.points.push_back(pt(fx-wl/2,  hw + ww, bz)); m.points.push_back(pt(fx+wl/2,  hw + ww, bz));
    m.points.push_back(pt(fx-wl/2,  hw - ww, bz)); m.points.push_back(pt(fx-wl/2, hw + ww, bz));
    m.points.push_back(pt(fx+wl/2,  hw - ww, bz)); m.points.push_back(pt(fx+wl/2, hw + ww, bz));

    footprint_pub_->publish(m);
}

// ─────────────────────────────────────────────────────────────────────────────
// Predicted path — line + spheres at each step for clarity
// ─────────────────────────────────────────────────────────────────────────────
void MPCVisualizer::publishPredictedPath()
{
    if (!has_pred_path_ || predicted_path_->poses.empty()) return;

    visualization_msgs::msg::MarkerArray ma;
    auto stamp = now();

    // Predicted trajectory line
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp    = stamp;
    line.ns   = "predicted_line";
    line.id   = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.08;
    line.color.r = 1.0f; line.color.g = 0.0f; line.color.b = 0.8f; line.color.a = 1.0f;
    line.pose.orientation.w = 1.0;

    // Step markers (spheres)
    visualization_msgs::msg::Marker spheres;
    spheres.header.frame_id = "map";
    spheres.header.stamp    = stamp;
    spheres.ns   = "predicted_steps";
    spheres.id   = 1;
    spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::msg::Marker::ADD;
    spheres.scale.x = 0.18; spheres.scale.y = 0.18; spheres.scale.z = 0.18;
    spheres.color.r = 1.0f; spheres.color.g = 0.2f; spheres.color.b = 1.0f; spheres.color.a = 0.9f;
    spheres.pose.orientation.w = 1.0;

    for (const auto& ps : predicted_path_->poses) {
        geometry_msgs::msg::Point p;
        p.x = ps.pose.position.x;
        p.y = ps.pose.position.y;
        p.z = 0.1;
        line.points.push_back(p);
        spheres.points.push_back(p);
    }

    ma.markers.push_back(line);
    ma.markers.push_back(spheres);
    pred_viz_pub_->publish(ma);
}

// ─────────────────────────────────────────────────────────────────────────────
// Track constraints — project each predicted pose onto ArcSpline, then draw
// the exact constraint line from TrackConstraints at that arc-length position.
// ─────────────────────────────────────────────────────────────────────────────
void MPCVisualizer::publishTrackConstraints()
{
    if (!has_spline_) return;
    if (!has_pred_path_ || predicted_path_->poses.empty()) return;

    visualization_msgs::msg::MarkerArray ma;
    auto stamp = now();

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp    = stamp;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    del.ns     = "constraints";
    ma.markers.push_back(del);

    for (size_t i = 0; i < predicted_path_->poses.size(); ++i) {
        const double px = predicted_path_->poses[i].pose.position.x;
        const double py = predicted_path_->poses[i].pose.position.y;

        // Project onto fitted spline → arc-length s
        const double s = track_spline_.projectOntoSpline(Eigen::Vector2d(px, py));

        // TrackConstraints gives the normal direction and boundary geometry at s
        mpc_controller::TrackConstraint tc =
            track_constraints_.getTrackConstraints(track_spline_, s);

        Eigen::Vector2d pos_c = track_spline_.getPosition(s);
        const double nx = tc.C(0, 0);  // left-of-travel normal
        const double ny = tc.C(0, 1);

        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp    = stamp;
        line.ns   = "constraints";
        line.id   = static_cast<int>(i);
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.04;
        line.color.r = 1.0f; line.color.g = 0.5f; line.color.b = 0.0f; line.color.a = 0.6f;
        line.pose.orientation.w = 1.0;

        // Inner boundary (right of travel) ↔ outer boundary (left of travel)
        geometry_msgs::msg::Point p1, p2;
        p1.x = pos_c(0) - r_inner_ * nx;  p1.y = pos_c(1) - r_inner_ * ny;  p1.z = 0.05;
        p2.x = pos_c(0) + r_outer_ * nx;  p2.y = pos_c(1) + r_outer_ * ny;  p2.z = 0.05;
        line.points.push_back(p1);
        line.points.push_back(p2);
        ma.markers.push_back(line);
    }

    constraint_pub_->publish(ma);
}

// ─────────────────────────────────────────────────────────────────────────────
// TF: map → base_link
// ─────────────────────────────────────────────────────────────────────────────
void MPCVisualizer::broadcastTF(const nav_msgs::msg::Odometry& odom)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = now();
    t.header.frame_id = "map";
    t.child_frame_id  = "base_link";
    t.transform.translation.x = odom.pose.pose.position.x;
    t.transform.translation.y = odom.pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation      = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
}
