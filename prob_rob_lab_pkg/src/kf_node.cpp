#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "prob_rob_lab_pkg/motion_models.hpp"
#include "prob_rob_lab_pkg/filter_math.hpp"

using namespace prob_rob_lab;

class KalmanFilterNode : public rclcpp::Node
{
public:
  KalmanFilterNode() : Node("kf_node")
  {
    this->declare_parameter<std::string>("motion_model", "differential_drive");
    this->declare_parameter<double>("q_scale", 1.0);
    this->declare_parameter<double>("r_x",  0.01);
    this->declare_parameter<double>("r_y",  0.01);
    this->declare_parameter<double>("r_th", 0.001);

    std::string model = this->get_parameter("motion_model").as_string();
    q_scale_          = this->get_parameter("q_scale").as_double();
    use_diff_drive_   = (model == "differential_drive");

    RCLCPP_INFO(this->get_logger(), "[KF] Motion model: %s",
      use_diff_drive_ ? DifferentialDriveModel::NAME : SimpleMotionModel::NAME);

    double rx  = this->get_parameter("r_x").as_double();
    double ry  = this->get_parameter("r_y").as_double();
    double rth = this->get_parameter("r_th").as_double();
    R_ = {{{rx,0,0},{0,ry,0},{0,0,rth}}};

    mu_    = {0.0, 0.0, 0.0};
    Sigma_ = {{{1,0,0},{0,1,0},{0,0,1}}};

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/kf/pose", 10);
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      std::bind(&KalmanFilterNode::odom_callback, this, std::placeholders::_1));
    sub_cmd_  = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_unstamped", 10,
      std::bind(&KalmanFilterNode::cmd_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[KF] Node started.");
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    u_[0] = msg->linear.x;
    u_[1] = msg->angular.z;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if (!initialized_) {
      mu_[0] = msg->pose.pose.position.x;
      mu_[1] = msg->pose.pose.position.y;
      mu_[2] = yaw_from_odom(*msg);
      Sigma_ = {{{0.01,0,0},{0,0.01,0},{0,0,0.001}}};
      last_t_ = t; initialized_ = true; return;
    }
    double dt = t - last_t_;
    if (dt <= 0.0) return;
    last_t_ = t;

    Mat3 F = use_diff_drive_ ? diff_model_.jacobian_F(mu_,u_,dt) : simple_model_.jacobian_F(mu_,u_,dt);
    Vec3 mu_pred = use_diff_drive_ ? diff_model_.predict(mu_,u_,dt) : simple_model_.predict(mu_,u_,dt);
    Mat3 Q = use_diff_drive_ ? diff_model_.process_noise_Q(u_,dt,q_scale_) : simple_model_.process_noise_Q(u_,dt,q_scale_);
    Mat3 Ft = mat3_transpose(F);
    Mat3 Sig_pred = mat3_add(mat3_mul(mat3_mul(F,Sigma_),Ft), Q);

    Vec3 z = {msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_from_odom(*msg)};
    Vec3 innov = {z[0]-mu_pred[0], z[1]-mu_pred[1], wrap_angle(z[2]-mu_pred[2])};
    Mat3 S = mat3_add(Sig_pred, R_);
    Mat3 K = mat3_mul(Sig_pred, mat3_inv(S));

    mu_[0] = mu_pred[0] + K[0][0]*innov[0] + K[0][1]*innov[1] + K[0][2]*innov[2];
    mu_[1] = mu_pred[1] + K[1][0]*innov[0] + K[1][1]*innov[1] + K[1][2]*innov[2];
    mu_[2] = wrap_angle(mu_pred[2] + K[2][0]*innov[0] + K[2][1]*innov[1] + K[2][2]*innov[2]);
    Sigma_ = joseph_update(K, {{{1,0,0},{0,1,0},{0,0,1}}}, Sig_pred);

    publish_pose(msg->header.stamp);
  }

  double yaw_from_odom(const nav_msgs::msg::Odometry & msg)
  {
    tf2::Quaternion q; tf2::fromMsg(msg.pose.pose.orientation, q);
    double r, p, y; tf2::Matrix3x3(q).getRPY(r,p,y); return y;
  }

  void publish_pose(const rclcpp::Time & stamp)
  {
    auto out = geometry_msgs::msg::PoseWithCovarianceStamped();
    out.header.stamp = stamp; out.header.frame_id = "odom";
    out.pose.pose.position.x = mu_[0]; out.pose.pose.position.y = mu_[1];
    tf2::Quaternion q; q.setRPY(0,0,mu_[2]); out.pose.pose.orientation = tf2::toMsg(q);
    out.pose.covariance[0] = Sigma_[0][0]; out.pose.covariance[7] = Sigma_[1][1]; out.pose.covariance[35] = Sigma_[2][2];
    pub_pose_->publish(out);
  }

  bool initialized_=false; bool use_diff_drive_=true;
  double last_t_=0.0; double q_scale_=1.0;
  Vec3 mu_={0,0,0}; Vec2 u_={0,0}; Mat3 Sigma_; Mat3 R_;
  SimpleMotionModel simple_model_; DifferentialDriveModel diff_model_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilterNode>());
  rclcpp::shutdown(); return 0;
}
