// TwistCmdController.cc
// Gazebo Sim system plugin that applies Twist commands to a model link.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>
#include <sdf/Cylinder.hh>

namespace cable_robot_controller
{
using namespace gz;
using namespace sim;

class TwistCmdController:
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
  private: struct CablePoint
  {
    int idx{0};
    math::Vector3d pos;
    double radius{0.12};
  };

  private: struct CableProjection
  {
    bool valid{false};
    math::Vector3d point{0, 0, 0};
    double radius{0.12};
  };

  public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "[TwistCmdController] Invalid model entity.\n";
      return;
    }

    this->linkName = this->GetSdfStr(_sdf, "link_name", "base_link");
    this->cmdTopic = this->GetSdfStr(_sdf, "cmd_topic", "/model/cable_repair_robot/cmd_vel");
    this->inspectionCompleteTopic = this->GetSdfStr(
      _sdf,
      "inspection_complete_topic",
      "/model/cable_repair_robot/inspection_complete");
    this->forceVelGain = this->GetSdfD(_sdf, "force_vel_gain", 9.0);
    this->torqueAngGain = this->GetSdfD(_sdf, "torque_ang_gain", 7.0);
    this->maxForce = this->GetSdfD(_sdf, "max_force", 180.0);
    this->maxTorque = this->GetSdfD(_sdf, "max_torque", 65.0);
    this->cmdTimeoutSec = this->GetSdfD(_sdf, "cmd_timeout_sec", 0.5);
    this->robotCableClearance =
      this->GetSdfD(_sdf, "robot_cable_clearance", 0.20);
    this->robotHalfHeight = this->GetSdfD(_sdf, "robot_half_height", 0.15);
    this->verticalKp = this->GetSdfD(_sdf, "vertical_kp", 1.4);
    this->verticalDeadband = this->GetSdfD(_sdf, "vertical_deadband", 0.015);
    this->maxVerticalSpeed = this->GetSdfD(_sdf, "max_vertical_speed", 0.25);
    this->verticalSmoothingTau =
      this->GetSdfD(_sdf, "vertical_smoothing_tau", 0.45);
    this->cableMapRefreshSec =
      this->GetSdfD(_sdf, "cable_map_refresh_sec", 0.5);
    this->stopMarkerName =
      this->GetSdfStr(_sdf, "stop_marker_name", "stop_marker");
    this->stopMarkerReachDistance =
      this->GetSdfD(_sdf, "stop_marker_reach_distance", 1.0);

    this->baseLink = this->model.LinkByName(_ecm, this->linkName);
    if (this->baseLink == kNullEntity)
    {
      gzerr << "[TwistCmdController] Link not found: " << this->linkName << "\n";
      return;
    }

    this->BuildCableMap(_ecm);
    this->node.Subscribe(this->cmdTopic, &TwistCmdController::OnCmd, this);
    this->node.Subscribe(
      this->inspectionCompleteTopic,
      &TwistCmdController::OnInspectionComplete,
      this);
    this->configured = true;

    gzmsg << "[TwistCmdController] Loaded. link=" << this->linkName
          << " cmdTopic=" << this->cmdTopic
          << " stopTopic=" << this->inspectionCompleteTopic
          << " cablePoints=" << this->cablePoints.size()
          << " clearance=" << this->robotCableClearance << "\n";
  }

  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override
  {
    this->activeEcm = &_ecm;
    if (!this->configured || _info.paused || this->baseLink == kNullEntity)
      return;

    if (!this->inspectionComplete.load())
      this->MaybeStopAtMarker(_ecm);

    if (this->inspectionComplete.load())
    {
      this->LockModelStatic(_ecm);
      if (!this->inspectionStopPoseCaptured)
      {
        this->inspectionStopPose = worldPose(this->model.Entity(), _ecm);
        this->inspectionStopPoseCaptured = true;
      }

      this->ForceStopMotion(this->baseLink);
      _ecm.RemoveComponent<components::WorldPoseCmd>(this->model.Entity());
      _ecm.CreateComponent(
        this->model.Entity(),
        components::WorldPoseCmd(this->inspectionStopPose));
      return;
    }
    else if (this->inspectionStopPoseCaptured)
    {
      _ecm.RemoveComponent<components::WorldPoseCmd>(this->model.Entity());
      this->inspectionStopPoseCaptured = false;
    }

    if (this->cablePoints.empty() ||
        this->lastCableMapRefreshSimTime == std::chrono::steady_clock::duration::zero() ||
        std::chrono::duration<double>(
          _info.simTime - this->lastCableMapRefreshSimTime).count() >= this->cableMapRefreshSec)
    {
      this->BuildCableMap(_ecm);
      this->lastCableMapRefreshSimTime = _info.simTime;
    }

    math::Vector3d linCmd = math::Vector3d::Zero;
    math::Vector3d angCmd = math::Vector3d::Zero;
    {
      std::lock_guard<std::mutex> lock(this->cmdMutex);
      const double ageSec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - this->lastCmdWallTime).count();
      if (ageSec <= this->cmdTimeoutSec)
      {
        linCmd = this->lastLinearCmd;
        angCmd = this->lastAngularCmd;
      }
    }

    const double dtSec =
      std::max(0.0, std::chrono::duration<double>(_info.dt).count());
    this->ApplyCmd(linCmd, angCmd, dtSec);
  }

  private: void OnCmd(const msgs::Twist &_msg)
  {
    if (this->inspectionComplete.load())
      return;

    std::lock_guard<std::mutex> lock(this->cmdMutex);
    this->lastLinearCmd.Set(
      _msg.linear().x(),
      _msg.linear().y(),
      _msg.linear().z());
    this->lastAngularCmd.Set(
      _msg.angular().x(),
      _msg.angular().y(),
      _msg.angular().z());
    this->lastCmdWallTime = std::chrono::steady_clock::now();
  }

  private: void OnInspectionComplete(const msgs::Boolean &_msg)
  {
    this->inspectionComplete.store(_msg.data());
    if (!_msg.data())
      return;

    this->LockStoredCommand();
  }

  private: void LockStoredCommand()
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    this->lastLinearCmd = math::Vector3d::Zero;
    this->lastAngularCmd = math::Vector3d::Zero;
    this->lastCmdWallTime = std::chrono::steady_clock::time_point{};
  }

  private: void LockModelStatic(EntityComponentManager &_ecm)
  {
    if (this->modelStaticApplied)
      return;

    _ecm.RemoveComponent<components::Static>(this->model.Entity());
    _ecm.CreateComponent(this->model.Entity(), components::Static(true));
    this->modelStaticApplied = true;
  }

  private: void UpdateStopMarker(EntityComponentManager &_ecm)
  {
    if (this->stopMarkerEntity != kNullEntity)
      return;

    _ecm.Each<components::Name, components::Pose>(
      [&](const Entity &_e, const components::Name *_name,
          const components::Pose *) -> bool
      {
        if (_name && _name->Data() == this->stopMarkerName)
        {
          this->stopMarkerEntity = _e;
          return false;
        }
        return true;
      });
  }

  private: void MaybeStopAtMarker(EntityComponentManager &_ecm)
  {
    this->UpdateStopMarker(_ecm);
    if (this->stopMarkerEntity == kNullEntity)
      return;

    const math::Vector3d robotPos = worldPose(this->model.Entity(), _ecm).Pos();
    const math::Vector3d markerPos = worldPose(this->stopMarkerEntity, _ecm).Pos();
    if ((robotPos - markerPos).Length() >= this->stopMarkerReachDistance)
      return;

    this->inspectionComplete.store(true);
    this->LockStoredCommand();
    this->LockModelStatic(_ecm);
    this->ForceStopMotion(this->baseLink);
  }

  private: void ForceStopMotion(const Entity &_bodyEntity)
  {
    if (this->activeEcm == nullptr || _bodyEntity == kNullEntity)
      return;

    this->activeEcm->RemoveComponent<components::LinearVelocityCmd>(_bodyEntity);
    this->activeEcm->RemoveComponent<components::AngularVelocityCmd>(_bodyEntity);
    this->activeEcm->RemoveComponent<components::LinearVelocityCmd>(this->model.Entity());
    this->activeEcm->RemoveComponent<components::AngularVelocityCmd>(this->model.Entity());

    this->activeEcm->CreateComponent(
      _bodyEntity,
      components::LinearVelocityCmd(math::Vector3d(0, 0, 0)));
    this->activeEcm->CreateComponent(
      _bodyEntity,
      components::AngularVelocityCmd(math::Vector3d(0, 0, 0)));
  }

  private: void ApplyCmd(
      const math::Vector3d &_linear,
      const math::Vector3d &_angular,
      double _dtSec)
  {
    if (this->activeEcm == nullptr || this->baseLink == kNullEntity)
      return;

    const math::Pose3d basePose = worldPose(this->baseLink, *this->activeEcm);
    const math::Vector3d desLinWorld = basePose.Rot().RotateVector(_linear);
    const double verticalCmd =
      this->ComputeVerticalVelocity(basePose.Pos().Z(), basePose.Pos(), _dtSec);
    math::Vector3d commandedLinWorld = desLinWorld;
    commandedLinWorld.Z(commandedLinWorld.Z() + verticalCmd);
    const math::Vector3d desAngWorld = basePose.Rot().RotateVector(_angular);

    this->activeEcm->RemoveComponent<components::LinearVelocityCmd>(this->baseLink);
    this->activeEcm->RemoveComponent<components::AngularVelocityCmd>(this->baseLink);
    this->activeEcm->RemoveComponent<components::LinearVelocityCmd>(this->model.Entity());
    this->activeEcm->RemoveComponent<components::AngularVelocityCmd>(this->model.Entity());

    this->activeEcm->CreateComponent(
      this->baseLink,
      components::LinearVelocityCmd(commandedLinWorld));
    this->activeEcm->CreateComponent(
      this->baseLink,
      components::AngularVelocityCmd(desAngWorld));
  }

  private: void BuildCableMap(EntityComponentManager &_ecm)
  {
    this->cablePoints.clear();

    _ecm.Each<components::Name, components::Collision, components::Pose, components::Geometry>(
      [&](const Entity &_e, const components::Name *_name, const components::Collision *,
          const components::Pose *, const components::Geometry *_geom) -> bool
      {
        if (!_name || !_geom)
          return true;

        const std::string n = _name->Data();
        const std::string prefix = "segment_collision_";
        if (n.rfind(prefix, 0) != 0)
          return true;

        int idx = 0;
        try
        {
          idx = std::stoi(n.substr(prefix.size()));
        }
        catch (...)
        {
          return true;
        }

        const sdf::Geometry &geom = _geom->Data();
        if (geom.Type() != sdf::GeometryType::CYLINDER || !geom.CylinderShape())
          return true;

        CablePoint point;
        point.idx = idx;
        point.pos = worldPose(_e, _ecm).Pos();
        point.radius = geom.CylinderShape()->Radius();
        this->cablePoints.push_back(point);
        return true;
      });

    std::sort(
      this->cablePoints.begin(),
      this->cablePoints.end(),
      [](const CablePoint &_a, const CablePoint &_b)
      {
        return _a.idx < _b.idx;
      });
  }

  private: CableProjection ProjectOntoCable(const math::Vector3d &_robotPos) const
  {
    CableProjection best;
    if (this->cablePoints.empty())
      return best;

    if (this->cablePoints.size() == 1u)
    {
      best.valid = true;
      best.point = this->cablePoints.front().pos;
      best.radius = this->cablePoints.front().radius;
      return best;
    }

    const double rx = _robotPos.X();
    const double ry = _robotPos.Y();
    double bestDistSq = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i + 1 < this->cablePoints.size(); ++i)
    {
      const auto &a = this->cablePoints[i];
      const auto &b = this->cablePoints[i + 1];
      const double dx = b.pos.X() - a.pos.X();
      const double dy = b.pos.Y() - a.pos.Y();
      const double segLenSq = (dx * dx) + (dy * dy);
      if (segLenSq <= 1e-9)
        continue;

      const double t = std::clamp(
        (((rx - a.pos.X()) * dx) + ((ry - a.pos.Y()) * dy)) / segLenSq,
        0.0,
        1.0);
      const math::Vector3d point = a.pos + ((b.pos - a.pos) * t);
      const double ex = rx - point.X();
      const double ey = ry - point.Y();
      const double distSq = (ex * ex) + (ey * ey);

      if (distSq < bestDistSq)
      {
        bestDistSq = distSq;
        best.valid = true;
        best.point = point;
        best.radius = a.radius + ((b.radius - a.radius) * t);
      }
    }

    if (!best.valid)
    {
      const auto &fallback = this->cablePoints.front();
      best.valid = true;
      best.point = fallback.pos;
      best.radius = fallback.radius;
    }

    return best;
  }

  private: double ComputeVerticalVelocity(
      double _robotZ,
      const math::Vector3d &_robotPos,
      double _dtSec)
  {
    const CableProjection projection = this->ProjectOntoCable(_robotPos);
    if (!projection.valid)
    {
      return this->SmoothVerticalVelocity(0.0, _dtSec);
    }

    // Maintain clearance between the robot underside and the cable surface.
    const double desiredRobotZ =
      projection.point.Z() + projection.radius + this->robotHalfHeight +
      this->robotCableClearance;
    const double zError = desiredRobotZ - _robotZ;

    double targetVerticalVel = 0.0;
    if (std::abs(zError) > this->verticalDeadband)
    {
      targetVerticalVel = std::clamp(
        this->verticalKp * zError,
        -this->maxVerticalSpeed,
        this->maxVerticalSpeed);
    }

    return this->SmoothVerticalVelocity(targetVerticalVel, _dtSec);
  }

  private: double SmoothVerticalVelocity(double _targetVel, double _dtSec)
  {
    if (_dtSec <= 1e-6)
    {
      this->smoothedVerticalVelocity = _targetVel;
      return this->smoothedVerticalVelocity;
    }

    const double tau = std::max(1e-3, this->verticalSmoothingTau);
    const double alpha = std::clamp(1.0 - std::exp(-_dtSec / tau), 0.0, 1.0);
    this->smoothedVerticalVelocity +=
      alpha * (_targetVel - this->smoothedVerticalVelocity);
    return this->smoothedVerticalVelocity;
  }

  private: std::string GetSdfStr(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      const std::string &_def) const
  {
    if (_sdf && _sdf->HasElement(_name))
      return _sdf->Get<std::string>(_name);
    return _def;
  }

  private: double GetSdfD(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      double _def) const
  {
    if (_sdf && _sdf->HasElement(_name))
      return _sdf->Get<double>(_name);
    return _def;
  }

  private: Model model{kNullEntity};
  private: Entity baseLink{kNullEntity};
  private: EntityComponentManager *activeEcm{nullptr};
  private: transport::Node node;
  private: std::vector<CablePoint> cablePoints;

  private: std::mutex cmdMutex;
  private: math::Vector3d lastLinearCmd{0, 0, 0};
  private: math::Vector3d lastAngularCmd{0, 0, 0};
  private: std::chrono::steady_clock::time_point lastCmdWallTime{};
  private: std::chrono::steady_clock::duration lastCableMapRefreshSimTime{};

  private: bool configured{false};
  private: std::string linkName{"base_link"};
  private: std::string cmdTopic{"/model/cable_repair_robot/cmd_vel"};
  private: std::string inspectionCompleteTopic{
    "/model/cable_repair_robot/inspection_complete"};
  private: double forceVelGain{9.0};
  private: double torqueAngGain{7.0};
  private: double maxForce{180.0};
  private: double maxTorque{65.0};
  private: double cmdTimeoutSec{0.5};
  private: double robotCableClearance{0.20};
  private: double robotHalfHeight{0.15};
  private: double verticalKp{1.4};
  private: double verticalDeadband{0.015};
  private: double maxVerticalSpeed{0.25};
  private: double verticalSmoothingTau{0.45};
  private: double cableMapRefreshSec{0.5};
  private: std::string stopMarkerName{"stop_marker"};
  private: double stopMarkerReachDistance{1.0};
  private: double smoothedVerticalVelocity{0.0};
  private: std::atomic_bool inspectionComplete{false};
  private: math::Pose3d inspectionStopPose;
  private: bool inspectionStopPoseCaptured{false};
  private: Entity stopMarkerEntity{kNullEntity};
  private: bool modelStaticApplied{false};
};

}  // namespace cable_robot_controller

GZ_ADD_PLUGIN(
  cable_robot_controller::TwistCmdController,
  gz::sim::System,
  cable_robot_controller::TwistCmdController::ISystemConfigure,
  cable_robot_controller::TwistCmdController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(cable_robot_controller::TwistCmdController, "cable_robot_controller::TwistCmdController")
GZ_ADD_PLUGIN_ALIAS(cable_robot_controller::TwistCmdController, "cable_robot_controller_twist")
