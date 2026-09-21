#ifndef PIPER_PLATFORM_GUI__PIPER_PLATFORM_GUI_HH_
#define PIPER_PLATFORM_GUI__PIPER_PLATFORM_GUI_HH_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gz/gui/Plugin.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace piper_platform_gui
{

/// Gazebo GUI panel that jogs the tunnel_hil_sim movable platform.
///
/// This plugin is deliberately thin: all clamping, continuous command
/// republishing and collision/settle detection live in the ROS-side
/// platform_control_node (Python, easy to test on its own). This plugin
/// only (a) lets the user type/drag a target Y/Z and press
/// Apply/Center/Reset, which are relayed as ROS parameter updates on that
/// node, and (b) shows the live current Y/Z read back from /sim/joint_states
/// so the panel also works as a requested-vs-actual position readout.
///
/// It talks to ROS only through /sim-only interfaces (platform_control_node's
/// parameters, /sim/joint_states) - it has no path to any real-hardware
/// topic.
class PiperPlatformGui : public gz::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(double targetY READ TargetY WRITE SetTargetY NOTIFY TargetYChanged)
  Q_PROPERTY(double targetZ READ TargetZ WRITE SetTargetZ NOTIFY TargetZChanged)
  Q_PROPERTY(double currentY READ CurrentY NOTIFY CurrentYChanged)
  Q_PROPERTY(double currentZ READ CurrentZ NOTIFY CurrentZChanged)
  Q_PROPERTY(double minY READ MinY CONSTANT)
  Q_PROPERTY(double maxY READ MaxY CONSTANT)
  Q_PROPERTY(double minZ READ MinZ CONSTANT)
  Q_PROPERTY(double maxZ READ MaxZ CONSTANT)
  Q_PROPERTY(QString statusMessage READ StatusMessage NOTIFY StatusMessageChanged)

  public: PiperPlatformGui();

  public: ~PiperPlatformGui() override;

  protected: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  public: double TargetY() const { return this->targetY; }
  public: void SetTargetY(double _y);
  public: double TargetZ() const { return this->targetZ; }
  public: void SetTargetZ(double _z);
  public: double CurrentY() const { return this->currentY; }
  public: double CurrentZ() const { return this->currentZ; }
  public: double MinY() const { return this->minY; }
  public: double MaxY() const { return this->maxY; }
  public: double MinZ() const { return this->minZ; }
  public: double MaxZ() const { return this->maxZ; }
  public: QString StatusMessage() const { return this->statusMessage; }

  public slots: void Apply();
  public slots: void Center();
  public slots: void Reset();

  signals:
  void TargetYChanged();
  void TargetZChanged();
  void CurrentYChanged();
  void CurrentZChanged();
  void StatusMessageChanged();

  private: void PublishTargetParams(bool _setY, bool _setZ);
  private: void OnJointState(const sensor_msgs::msg::JointState::SharedPtr _msg);
  private: void UpdateStatus();

  private: double targetY = 0.0;
  private: double targetZ = 1.0;
  private: double currentY = 0.0;
  private: double currentZ = 1.0;
  private: double minY = -4.0;
  private: double maxY = 4.0;
  private: double minZ = 0.5;
  private: double maxZ = 7.0;
  private: double initialY = 0.0;
  private: double initialZ = 1.0;
  private: QString statusMessage = "";

  private: std::string lateralJointName = "platform_lateral_joint";
  private: std::string liftJointName = "platform_lift_joint";
  private: std::string controlNodeName = "platform_control_node";
  private: std::chrono::steady_clock::time_point lastCommandTime =
      std::chrono::steady_clock::now();

  private: rclcpp::Node::SharedPtr node;
  private: rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub;
  private: rclcpp::AsyncParametersClient::SharedPtr paramsClient;
  private: std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  private: std::thread spinThread;
};

}  // namespace piper_platform_gui

#endif  // PIPER_PLATFORM_GUI__PIPER_PLATFORM_GUI_HH_
