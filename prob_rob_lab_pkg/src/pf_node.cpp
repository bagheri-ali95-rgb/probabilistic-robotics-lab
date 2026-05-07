#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <numeric>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "prob_rob_lab_pkg/motion_models.hpp"
#include "prob_rob_lab_pkg/filter_math.hpp"

using namespace prob_rob_lab;

static double gaussian_likelihood(const Vec3 & z, const Vec3 & mu, const Mat3 & R)
{
  Vec3 diff = {z[0]-mu[0], z[1]-mu[1], wrap_angle(z[2]-mu[2])};
  try {
    Mat3 Rinv = mat3_inv(R);
    double exp_val = 0.0;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        exp_val += diff[i] * Rinv[i][j] * diff[j];
    double det_R = mat3_det(R);
    double coeff = 1.0 / std::sqrt(std::pow(2*M_PI,3) * std::abs(det_R));
    return coeff * std::exp(-0.5 * exp_val);
  } catch (...) { return 1e-300; }
}

class ParticleFilterNode : public rclcpp::Node
{
public:
  ParticleFilterNode() : Node("pf_node"), rng_(std::random_device{}())
  {
    this->declare_parameter<std::string>("motion_model", "differential_drive");
    this->declare_parameter<int>("num_particles", 500);
    this->declare_parameter<double>("resample_threshold", 0.5);
    this->declare_parameter<double>("r_x",  0.01);
    this->declare_parameter<double>("r_y",  0.01);
    this->declare_parameter<double>("r_th", 0.001);

    std::string model = this->get_parameter("motion_model").as_string();
    N_              = this->get_parameter("num_particles").as_int();
    resample_thr_   = this->get_parameter("resample_threshold").as_double();
    use_diff_drive_ = (model == "differential_drive");

    double rx  = this->get_parameter("r_x").as_double();
    double ry  = this->get_parameter("r_y").as_double();
    double rth = this->get_parameter("r_th").as_double();
    R_ = {{{rx,0,0},{0,ry,0},{0,0,rth}}};

    particles_.resize(N_, {0.0,0.0,0.0});
    weights_.resize(N_, 1.0/N_);

    RCLCPP_INFO(this->get_logger(), "[PF] Motion model: %s | Particles: %d",
      use_diff_drive_ ? DifferentialDriveModel::NAME : SimpleMotionModel::NAME, N_);

    pub_pose_      = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pf/pose", 10);
    pub_particles_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pf/particles", 10);
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      std::bind(&ParticleFilterNode::odom_callback, this, std::placeholders::_1));
    sub_cmd_  = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_unstamped", 10,
      std::bind(&ParticleFilterNode::cmd_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[PF] Node started.");
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    u_[0] = msg->linear.x; u_[1] = msg->angular.z;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if (!initialized_) {
      double x0=msg->pose.pose.position.x, y0=msg->pose.pose.position.y, th0=yaw_from_odom(*msg);
      std::normal_distribution<double> nx(x0,0.1), ny(y0,0.1), nth(th0,0.05);
      for (int i=0;i<N_;i++) particles_[i]={nx(rng_),ny(rng_),nth(rng_)};
      std::fill(weights_.begin(),weights_.end(),1.0/N_);
      last_t_=t; initialized_=true; return;
    }
    double dt = t - last_t_;
    if (dt <= 0.0) return;
    last_t_ = t;

    for (int i=0;i<N_;i++) {
      particles_[i] = use_diff_drive_ ? diff_model_.sample(particles_[i],u_,dt) : simple_model_.sample(particles_[i],u_,dt);
      particles_[i][2] = wrap_angle(particles_[i][2]);
    }

    Vec3 z = {msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_from_odom(*msg)};
    for (int i=0;i<N_;i++) weights_[i] *= gaussian_likelihood(z, particles_[i], R_);

    double w_sum = std::accumulate(weights_.begin(),weights_.end(),0.0);
    if (w_sum < 1e-300) {
      RCLCPP_WARN(this->get_logger(),"[PF] Weight collapse – reinitialising.");
      std::fill(weights_.begin(),weights_.end(),1.0/N_);
    } else { for (auto & w : weights_) w /= w_sum; }

    double n_eff=0.0;
    for (auto w : weights_) n_eff += w*w;
    n_eff = 1.0/n_eff;
    if (n_eff < resample_thr_*N_) systematic_resample();

    publish_pose(msg->header.stamp);
    publish_particles(msg->header.stamp);
  }

  void systematic_resample()
  {
    std::vector<double> cumsum(N_);
    cumsum[0]=weights_[0];
    for (int i=1;i<N_;i++) cumsum[i]=cumsum[i-1]+weights_[i];
    double step=1.0/N_;
    std::uniform_real_distribution<double> uni(0.0,step);
    double start=uni(rng_);
    std::vector<Vec3> new_p(N_);
    int j=0;
    for (int i=0;i<N_;i++) {
      double pos=start+step*i;
      while (j<N_-1 && cumsum[j]<pos) j++;
      new_p[i]=particles_[j];
    }
    particles_=new_p;
    std::fill(weights_.begin(),weights_.end(),1.0/N_);
  }

  Vec3 mean_estimate()
  {
    double x=0,y=0,s=0,c=0;
    for (int i=0;i<N_;i++) {
      x+=weights_[i]*particles_[i][0]; y+=weights_[i]*particles_[i][1];
      s+=weights_[i]*std::sin(particles_[i][2]); c+=weights_[i]*std::cos(particles_[i][2]);
    }
    return {x,y,std::atan2(s,c)};
  }

  void publish_pose(const rclcpp::Time & stamp)
  {
    Vec3 mu=mean_estimate();
    double vx=0,vy=0,vt=0;
    for (int i=0;i<N_;i++) {
      vx+=weights_[i]*(particles_[i][0]-mu[0])*(particles_[i][0]-mu[0]);
      vy+=weights_[i]*(particles_[i][1]-mu[1])*(particles_[i][1]-mu[1]);
      double dth=wrap_angle(particles_[i][2]-mu[2]);
      vt+=weights_[i]*dth*dth;
    }
    auto out=geometry_msgs::msg::PoseWithCovarianceStamped();
    out.header.stamp=stamp; out.header.frame_id="odom";
    out.pose.pose.position.x=mu[0]; out.pose.pose.position.y=mu[1];
    tf2::Quaternion q; q.setRPY(0,0,mu[2]); out.pose.pose.orientation=tf2::toMsg(q);
    out.pose.covariance[0]=vx; out.pose.covariance[7]=vy; out.pose.covariance[35]=vt;
    pub_pose_->publish(out);
  }

  void publish_particles(const rclcpp::Time & stamp)
  {
    auto msg=geometry_msgs::msg::PoseArray();
    msg.header.stamp=stamp; msg.header.frame_id="odom";
    for (int i=0;i<N_;i++) {
      geometry_msgs::msg::Pose p;
      p.position.x=particles_[i][0]; p.position.y=particles_[i][1];
      tf2::Quaternion q; q.setRPY(0,0,particles_[i][2]); p.orientation=tf2::toMsg(q);
      msg.poses.push_back(p);
    }
    pub_particles_->publish(msg);
  }

  double yaw_from_odom(const nav_msgs::msg::Odometry & msg)
  {
    tf2::Quaternion q; tf2::fromMsg(msg.pose.pose.orientation,q);
    double r,p,y; tf2::Matrix3x3(q).getRPY(r,p,y); return y;
  }

  bool initialized_=false; bool use_diff_drive_=true;
  double last_t_=0.0; int N_=500; double resample_thr_=0.5;
  Vec2 u_={0,0}; Mat3 R_;
  std::vector<Vec3> particles_; std::vector<double> weights_;
  std::mt19937 rng_;
  SimpleMotionModel simple_model_; DifferentialDriveModel diff_model_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particles_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParticleFilterNode>());
  rclcpp::shutdown(); return 0;
}
