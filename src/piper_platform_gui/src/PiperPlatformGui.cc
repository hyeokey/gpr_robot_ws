#include "piper_platform_gui/PiperPlatformGui.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gz/plugin/Register.hh>

namespace piper_platform_gui
{

namespace
{
double ReadDouble(const tinyxml2::XMLElement *_elem, const char *_tag, double _default)
{
  if (!_elem)
    return _default;
  auto *child = _elem->FirstChildElement(_tag);
  if (!child || !child->GetText())
    return _default;
  try
  {
    return std::stod(child->GetText());
  }
  catch (...)
  {
    return _default;
  }
}

std::string ReadString(
    const tinyxml2::XMLElement *_elem, const char *_tag, const std::string &_default)
{
  if (!_elem)
    return _default;
  auto *child = _elem->FirstChildElement(_tag);
  if (!child || !child->GetText())
    return _default;
  return std::string(child->GetText());
}
}  // namespace

PiperPlatformGui::PiperPlatformGui()
    : gz::gui::Plugin()
{
}

PiperPlatformGui::~PiperPlatformGui()
{
  if (this->executor)
    this->executor->cancel();
  if (this->spinThread.joinable())
    this->spinThread.join();
}

void PiperPlatformGui::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Piper Platform Control";

  this->minY = ReadDouble(_pluginElem, "min_y", this->minY);
  this->maxY = ReadDouble(_pluginElem, "max_y", this->maxY);
  this->minZ = ReadDouble(_pluginElem, "min_z", this->minZ);
  this->maxZ = ReadDouble(_pluginElem, "max_z", this->maxZ);
  this->initialY = ReadDouble(_pluginElem, "initial_y", this->initialY);
  this->initialZ = ReadDouble(_pluginElem, "initial_z", this->initialZ);
  this->lateralJointName = ReadString(_pluginElem, "lateral_joint", this->lateralJointName);
  this->liftJointName = ReadString(_pluginElem, "lift_joint", this->liftJointName);
  this->controlNodeName = ReadString(_pluginElem, "control_node", this->controlNodeName);
  std::string jointStatesTopic =
      ReadString(_pluginElem, "joint_states_topic", "/sim/joint_states");

  this->targetY = this->initialY;
  this->targetZ = this->initialZ;
  this->currentY = this->initialY;
  this->currentZ = this->initialZ;

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);
  this->node = std::make_shared<rclcpp::Node>("piper_platform_gui");

  this->jointStateSub = this->node->create_subscription<sensor_msgs::msg::JointState>(
      jointStatesTopic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr _msg)
      {
        this->OnJointState(_msg);
      });

  this->paramsClient =
      std::make_shared<rclcpp::AsyncParametersClient>(this->node, this->controlNodeName);

  this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor->add_node(this->node);
  this->spinThread = std::thread([this]() { this->executor->spin(); });
}

void PiperPlatformGui::SetTargetY(double _y)
{
  if (std::abs(_y - this->targetY) < 1e-9)
    return;
  this->targetY = _y;
  emit this->TargetYChanged();
}

void PiperPlatformGui::SetTargetZ(double _z)
{
  if (std::abs(_z - this->targetZ) < 1e-9)
    return;
  this->targetZ = _z;
  emit this->TargetZChanged();
}

void PiperPlatformGui::PublishTargetParams(bool _setY, bool _setZ)
{
  this->lastCommandTime = std::chrono::steady_clock::now();

  if (!this->paramsClient)
    return;
  if (!this->paramsClient->service_is_ready())
  {
    this->statusMessage = QString(
        "%1 parameter service not available yet")
        .arg(QString::fromStdString(this->controlNodeName));
    emit this->StatusMessageChanged();
    return;
  }

  std::vector<rclcpp::Parameter> params;
  if (_setY)
    params.emplace_back("target_y", this->targetY);
  if (_setZ)
    params.emplace_back("target_z", this->targetZ);
  if (!params.empty())
    this->paramsClient->set_parameters(params);
}

void PiperPlatformGui::Apply()
{
  double clampedY = std::clamp(this->targetY, this->minY, this->maxY);
  double clampedZ = std::clamp(this->targetZ, this->minZ, this->maxZ);
  if (clampedY != this->targetY)
  {
    this->targetY = clampedY;
    emit this->TargetYChanged();
  }
  if (clampedZ != this->targetZ)
  {
    this->targetZ = clampedZ;
    emit this->TargetZChanged();
  }
  this->PublishTargetParams(true, true);
}

void PiperPlatformGui::Center()
{
  // Lateral only returns to the middle of the range; height is left alone
  // (its parameter is simply not touched here).
  this->targetY = (this->minY + this->maxY) / 2.0;
  emit this->TargetYChanged();
  this->PublishTargetParams(true, false);
}

void PiperPlatformGui::Reset()
{
  this->targetY = this->initialY;
  this->targetZ = this->initialZ;
  emit this->TargetYChanged();
  emit this->TargetZChanged();
  this->PublishTargetParams(true, true);
}

void PiperPlatformGui::OnJointState(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  double y = std::numeric_limits<double>::quiet_NaN();
  double z = std::numeric_limits<double>::quiet_NaN();
  std::size_t count = std::min(_msg->name.size(), _msg->position.size());
  for (std::size_t i = 0; i < count; ++i)
  {
    if (_msg->name[i] == this->lateralJointName)
      y = _msg->position[i];
    else if (_msg->name[i] == this->liftJointName)
      z = _msg->position[i];
  }
  if (std::isnan(y) && std::isnan(z))
    return;

  QMetaObject::invokeMethod(
      this,
      [this, y, z]()
      {
        if (!std::isnan(y))
        {
          this->currentY = y;
          emit this->CurrentYChanged();
        }
        if (!std::isnan(z))
        {
          this->currentZ = z;
          emit this->CurrentZChanged();
        }
        this->UpdateStatus();
      },
      Qt::QueuedConnection);
}

void PiperPlatformGui::UpdateStatus()
{
  double errorY = this->currentY - this->targetY;
  double errorZ = this->currentZ - this->targetZ;
  auto elapsed = std::chrono::steady_clock::now() - this->lastCommandTime;
  constexpr double kTolM = 0.03;
  bool settleWindowPassed = elapsed > std::chrono::seconds(3);

  if (settleWindowPassed && (std::abs(errorY) > kTolM || std::abs(errorZ) > kTolM))
  {
    this->statusMessage = QString(
        "목표 도달 못함 (충돌/한계 의심) - 요청 Y=%1 Z=%2, 실제 Y=%3 Z=%4, 오차 Y=%5 Z=%6")
        .arg(this->targetY, 0, 'f', 3)
        .arg(this->targetZ, 0, 'f', 3)
        .arg(this->currentY, 0, 'f', 3)
        .arg(this->currentZ, 0, 'f', 3)
        .arg(errorY, 0, 'f', 3)
        .arg(errorZ, 0, 'f', 3);
  }
  else
  {
    this->statusMessage = "정상";
  }
  emit this->StatusMessageChanged();
}

}  // namespace piper_platform_gui

GZ_ADD_PLUGIN(
    piper_platform_gui::PiperPlatformGui,
    gz::gui::Plugin)
