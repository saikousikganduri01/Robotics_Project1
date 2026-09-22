// CableStateController.cc
// Gazebo Sim (gz-sim8) system plugin implementing 4-state cable search behavior.
//
// States:
// 1) SEARCH_CABLE
// 2) STOP_AT_CABLE
// 3) ALIGN_WITH_CABLE
// 4) FOLLOW_CABLE
//
// Control output topic (Twist): /model/<robot>/LinearVelocityCmd

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Cylinder.hh>

namespace cable_robot_controller
{
using namespace gz;
using namespace sim;

class CableStateController:
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
  enum class State
  {
    SEARCH_CABLE = 0,
    STOP_AT_CABLE = 1,
    ALIGN_WITH_CABLE = 2,
    FOLLOW_CABLE = 3,
    HALTED = 4
  };

  struct Vision
  {
    bool valid{false};
    bool cableDetected{false};
    double xErr{0.0};
    double yErr{0.0};
    double headingErr{0.0};
    double area{0.0};
    double lineAngle{0.0};
  };

  struct CablePoint
  {
    int idx{0};
    Entity visualEntity{kNullEntity};
    Entity collisionEntity{kNullEntity};
    math::Vector3d pos;
    math::Vector3d axis{0, 0, 1};
    math::Color color{0.01f, 0.01f, 0.01f, 1.0f};
    double radius{0.12};
    double length{0.0};
    bool isRed{false};
  };

  struct FaultInfo
  {
    std::string type;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double depth{0.0};
    math::Vector3d pos;
    int idx{-1};
    Entity visualEntity{kNullEntity};
    math::Vector3d endA;
    math::Vector3d endB;
    math::Color color{0.01f, 0.01f, 0.01f, 1.0f};
    double radius{0.12};
    int idxA{-1};
    int idxB{-1};
  };

  struct GapMeasure
  {
    double dist{0.0};
    math::Vector3d mid;
    math::Vector3d endA;
    math::Vector3d endB;
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
      gzerr << "[CableStateController] Invalid model entity.\\n";
      return;
    }

    this->robotName = this->GetSdfStr(_sdf, "robot_name", "cable_repair_robot");
    this->cableModelName = this->GetSdfStr(_sdf, "cable_model_name", "submarine_cable");
    this->cameraTopic = this->GetSdfStr(_sdf, "camera_topic", "/cable_repair_robot/camera");
    this->cmdTopic = this->GetSdfStr(_sdf, "linear_velocity_cmd_topic",
                                     "/model/cable_repair_robot/LinearVelocityCmd");
    this->targetZ = this->GetSdfD(_sdf, "target_z", 0.85);
    this->stopDistance = this->GetSdfD(_sdf, "stop_distance", 0.5);
    this->detectDistance = this->GetSdfD(_sdf, "detect_distance", 1.2);
    this->minSearchTime = this->GetSdfD(_sdf, "min_search_time", 5.0);
    this->faultStopSec = this->GetSdfD(_sdf, "fault_stop_sec", 0.8);
    this->stopAtCableSec = this->GetSdfD(_sdf, "stop_at_cable_sec", 0.1);
    this->maxVisionLossSec = this->GetSdfD(_sdf, "max_vision_loss_sec", 0.8);
    this->visionAreaMin = this->GetSdfD(_sdf, "vision_area_min", 0.00001);
    this->visionAreaMax = this->GetSdfD(_sdf, "vision_area_max", 0.60);
    this->visionFallbackLockDistance = this->GetSdfD(_sdf, "vision_fallback_lock_distance", 0.45);
    this->visionDarkLumThreshold = this->GetSdfD(_sdf, "vision_dark_luma_threshold", 85.0);
    this->visionRequiredForLock = this->GetSdfB(_sdf, "vision_required_for_lock", true);
    this->visionRequiredForRepair = this->GetSdfB(_sdf, "vision_required_for_repair", true);
    this->publishVelocityTopic = this->GetSdfB(_sdf, "publish_velocity_topic", false);
    this->forceVelGain = this->GetSdfD(_sdf, "force_vel_gain", 9.0);
    this->torqueAngGain = this->GetSdfD(_sdf, "torque_ang_gain", 7.0);
    this->maxForce = this->GetSdfD(_sdf, "max_force", 180.0);
    this->maxTorque = this->GetSdfD(_sdf, "max_torque", 65.0);
    this->cableMapRefreshSec = this->GetSdfD(_sdf, "cable_map_refresh_sec", 0.2);
    this->stopMarkerName = this->GetSdfStr(_sdf, "stop_marker_name", "stop_marker");
    this->stopMarkerReachDistance =
      this->GetSdfD(_sdf, "stop_marker_reach_distance", 1.0);

    // Optional fallback for corrosion segment ids, e.g. "18,75"
    this->ParseCorrosionSegments(this->GetSdfStr(_sdf, "corrosion_segments", "18,75"));

    this->node.Subscribe(this->cameraTopic, &CableStateController::OnImage, this);
    this->cmdPub = this->node.Advertise<msgs::Twist>(this->cmdTopic);
    this->baseLink = this->model.LinkByName(_ecm, "base_link");

    this->state = State::SEARCH_CABLE;
    this->stateEnter = std::chrono::steady_clock::now();
    this->configured = true;

    this->DiscoverWorldName(_ecm);

    // Build cable map once at startup (and lazily refresh if empty)
    this->BuildCableMap(_ecm);
    gzmsg << "[CableStateController] Loaded. robot=" << this->robotName
          << " cmdTopic=" << this->cmdTopic
          << " baseLinkFound=" << (this->baseLink != kNullEntity ? "yes" : "no")
          << " cablePoints=" << this->cablePoints.size() << "\n";
  }

  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override
  {
    this->activeEcm = &_ecm;
    if (this->baseLink == kNullEntity)
      this->baseLink = this->model.LinkByName(_ecm, "base_link");

    if (!this->configured || _info.paused)
      return;

    // Deterministic startup motion so the robot always begins moving.
    // This also helps diagnose plugin-load vs state-machine issues.
    if (_info.simTime < std::chrono::seconds(3))
    {
      this->PublishCmd(1.5, 0.0, 0.0, 0.0, 0.0, 0.20);
      return;
    }

    if (_info.simTime < this->faultStopUntil)
    {
      this->PublishStop();
      return;
    }

    if (this->cablePoints.empty())
    {
      this->BuildCableMap(_ecm);
      this->lastCableMapRefreshSimTime = _info.simTime;
    }
    else if (this->lastCableMapRefreshSimTime == std::chrono::steady_clock::duration::zero() ||
             std::chrono::duration<double>(_info.simTime - this->lastCableMapRefreshSimTime).count() >= this->cableMapRefreshSec)
    {
      this->BuildCableMap(_ecm);
      this->lastCableMapRefreshSimTime = _info.simTime;
    }

    const auto now = std::chrono::steady_clock::now();

    // Robot world pose
    const math::Pose3d robotPose = worldPose(this->model.Entity(), _ecm);
    const math::Vector3d robotPos = robotPose.Pos();
    const double robotYaw = robotPose.Rot().Yaw();
    this->UpdateStopMarker(_ecm);

    // Nearest cable point
    int nearestIdx = -1;
    math::Vector3d nearestPos;
    double cableDist = 1e9;
    this->NearestCablePoint(robotPos, nearestIdx, nearestPos, cableDist);

    if (this->inspectionCompleted)
    {
      this->LockRobotStatic(_ecm);
      if (!this->inspectionReportPrinted)
      {
        this->PrintStopMarkerReached();
        this->PrintInspectionReport();
        this->inspectionReportPrinted = true;
      }
      this->PublishStop();
      return;
    }

    if (this->ReachedStopMarker(robotPos))
    {
      this->inspectionCompleted = true;
      this->LockRobotStatic(_ecm);
      if (!this->inspectionReportPrinted)
      {
        this->PrintStopMarkerReached();
        this->PrintInspectionReport();
        this->inspectionReportPrinted = true;
      }
      this->PublishStop();
      this->Transition(State::HALTED);
      return;
    }

    // Vision snapshot
    Vision v;
    {
      std::lock_guard<std::mutex> lock(this->visionMtx);
      v = this->vision;
    }

    const double searchElapsedSec =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - this->stateEnter).count() / 1000.0;
    const bool inSearch = (this->state == State::SEARCH_CABLE);
    const bool visionEnabled = (searchElapsedSec >= this->minSearchTime);
    const bool cameraSeesCable = this->VisionConfident(v);
    if (cameraSeesCable)
    {
      this->lastVisionSeenSimTime = _info.simTime;
      if (_info.simTime - this->lastDetectionLogPrint > std::chrono::seconds(2))
      {
        this->lastDetectionLogPrint = _info.simTime;
        gzmsg << "[CableStateController] CABLE DETECTED BY CAMERA\n";
      }
    }
    else if (_info.simTime - this->lastDetectionLogPrint > std::chrono::seconds(2))
    {
        this->lastDetectionLogPrint = _info.simTime;
        gzmsg << "[CableStateController] Vision failed: "
              << "valid=" << v.valid
              << " detected=" << v.cableDetected
              << " area=" << v.area << " (min=" << this->visionAreaMin << ")"
              << " xErr=" << v.xErr
              << "\n";
    }
    const bool visionFresh = this->VisionFresh(_info.simTime);

    if (_info.simTime - this->lastDebugPrint > std::chrono::seconds(1))
    {
      this->lastDebugPrint = _info.simTime;
      gzmsg << "[CableStateController] state=" << this->StateName(this->state)
            << " pos=(" << robotPos.X() << "," << robotPos.Y() << "," << robotPos.Z() << ")"
            << " cableDist=" << cableDist
            << " nearestIdx=" << nearestIdx
            << " cam=" << (cameraSeesCable ? "1" : "0")
            << " baseLinkFound=" << (this->baseLink != kNullEntity ? "yes" : "no")
            << "\n";
    }

    // Camera-first lock condition.
    if (inSearch && visionEnabled && cameraSeesCable)
    {
      this->Transition(State::ALIGN_WITH_CABLE);
      std::cout << "\n****************************************" << std::endl;
      std::cout << " [CableStateController] CABLE DETECTED! " << std::endl;
      std::cout << "****************************************\n" << std::endl;
      return;
    }

    // Fallback lock only if explicitly allowed and robot is already very close.
    if (inSearch && visionEnabled && !this->visionRequiredForLock &&
        cableDist < this->detectDistance)
    {
      this->Transition(State::STOP_AT_CABLE);
      gzmsg << "CABLE DETECTED\\n";
      return;
    }

    // State machine implementation
    switch (this->state)
    {
      case State::SEARCH_CABLE:
      {
        // Search behavior: move forward more aggressively
        const double linX = 0.8;
        const double angZ = 0.2;
        const double linZ = std::clamp((this->targetZ - robotPos.Z()) * 0.8, -0.3, 0.3);
        this->PublishCmd(linX, 0.0, linZ, 0.0, 0.0, angZ);
        
        if (visionEnabled && cameraSeesCable)
        {
          std::cout << "[CableStateController] CABLE DETECTED! Stopping for 2s..." << std::endl;
          this->Transition(State::STOP_AT_CABLE);
        }
        break;
      }

      case State::STOP_AT_CABLE:
      {
        this->PublishStop();
        const double waitedSec = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->stateEnter).count() / 1000.0;
        
        if (waitedSec >= 2.0)
        {
          std::cout << "[CableStateController] Stop complete. Aligning (Rotation Only)..." << std::endl;
          this->Transition(State::ALIGN_WITH_CABLE);
        }
        break;
      }

      case State::ALIGN_WITH_CABLE:
      {
        // NO LINEAR MOTION (X/Y) as requested. Only Z for depth maintenance.
        const double angZ = cameraSeesCable ? this->VisionSteerCmd(v, 0.0, 2.0, 1.0) : 0.4;
        const double linZ = std::clamp((this->targetZ - robotPos.Z()) * 0.8, -0.2, 0.2);
        
        this->PublishCmd(0.0, 0.0, linZ, 0.0, 0.0, angZ);

        // Transition to follow when strictly aligned (heading error and center offset small)
        const bool aligned = cameraSeesCable && (std::abs(v.headingErr) < 0.04) && (std::abs(v.xErr) < 0.1);
        if (aligned)
        {
          std::cout << "[CableStateController] Aligned. Starting 2m/s Follow..." << std::endl;
          this->Transition(State::FOLLOW_CABLE);
        }
        break;
      }

      case State::FOLLOW_CABLE:
      {
        if (!cameraSeesCable && !visionFresh)
        {
          std::cout << "[CableStateController] Lost cable. Re-aligning..." << std::endl;
          this->Transition(State::ALIGN_WITH_CABLE);
          break;
        }

        // Fault detection
        std::optional<FaultInfo> fault = this->DetectFaultUnderRobot(robotPos);
        if (fault.has_value())
        {
          this->StoreFault(*fault);
          this->MarkFaultReported(*fault);
        }

        // Follow at 2.0 m/s as requested
        const double linX = 2.0;
        const double angZ = cameraSeesCable ? this->VisionSteerCmd(v, 2.0, 1.2, 1.0) : 0.0;
        const double linZ = std::clamp((this->targetZ - robotPos.Z()) * 0.8, -0.15, 0.15);

        this->PublishCmd(linX, 0.0, linZ, 0.0, 0.0, angZ);
        break;
      }

      case State::HALTED:
      {
        // Latch in stop state after a fault.
        this->PublishStop();
        break;
      }
    }
  }

  private: void OnImage(const msgs::Image &_msg)
  {
    static int imageCount = 0;
    if (++imageCount % 100 == 0) {
      gzmsg << "[CableStateController] Received 100 images from camera\n";
    }
    Vision out;
    out.valid = true;

    const int w = static_cast<int>(_msg.width());
    const int h = static_cast<int>(_msg.height());
    if (w <= 0 || h <= 0 || _msg.data().empty())
    {
      std::lock_guard<std::mutex> lock(this->visionMtx);
      this->vision = out;
      return;
    }

    const std::string &d = _msg.data();
    if (static_cast<int>(d.size()) < w * h * 3)
    {
      if (++imageCount % 100 == 0) {
        std::cerr << "[CableStateController] FATAL: Image data size mismatch! expected=" << (w*h*3) << " actual=" << d.size() << std::endl;
      }
      std::lock_guard<std::mutex> lock(this->visionMtx);
      this->vision = out;
      return;
    }

    int darkCount = 0;
    int nearCount = 0;
    int farCount = 0;
    int minX = w, minY = h, maxX = -1, maxY = -1;
    double sumX = 0, sumY = 0;
    double nearSumX = 0.0;
    double farSumX = 0.0;
    double sxx = 0, syy = 0, sxy = 0;

    const int roiTop = static_cast<int>(0.12 * h);
    const int roiBottom = static_cast<int>(0.97 * h);
    const int farBandTop = static_cast<int>(0.22 * h);
    const int farBandBottom = static_cast<int>(0.52 * h);
    const int nearBandTop = static_cast<int>(0.58 * h);
    const int nearBandBottom = static_cast<int>(0.92 * h);

    auto idx = [w](int x, int y) { return (y * w + x) * 3; };

    for (int y = roiTop; y < roiBottom; ++y)
    {
      for (int x = 0; x < w; ++x)
      {
        const int i = idx(x, y);
        const uint8_t r = static_cast<uint8_t>(d[i]);
        const uint8_t g = static_cast<uint8_t>(d[i + 1]);
        const uint8_t b = static_cast<uint8_t>(d[i + 2]);

        const int lum = (static_cast<int>(r) + static_cast<int>(g) + static_cast<int>(b)) / 3;
        const bool dark = (lum < this->visionDarkLumThreshold);  // black cable threshold

        if (dark)
        {
          darkCount++;
          minX = std::min(minX, x);
          minY = std::min(minY, y);
          maxX = std::max(maxX, x);
          maxY = std::max(maxY, y);

          sumX += x;
          sumY += y;
          sxx += static_cast<double>(x) * x;
          syy += static_cast<double>(y) * y;
          sxy += static_cast<double>(x) * y;

          if (y >= nearBandTop && y <= nearBandBottom)
          {
            nearCount++;
            nearSumX += x;
          }

          if (y >= farBandTop && y <= farBandBottom)
          {
            farCount++;
            farSumX += x;
          }
        }
      }
    }

    const double roiArea = static_cast<double>((roiBottom - roiTop) * w);
    out.area = roiArea > 0.0 ? static_cast<double>(darkCount) / roiArea : 0.0;

    if (darkCount > 20)
    {
      const double cx = sumX / darkCount;
      const double cy = sumY / darkCount;
      const double nearCx = (nearCount > 12) ? (nearSumX / nearCount) : cx;
      const double farCx = (farCount > 12) ? (farSumX / farCount) : cx;
      const double halfW = std::max(1.0, 0.5 * static_cast<double>(w));

      out.xErr = (nearCx - 0.5 * w) / halfW;
      out.yErr = (cy - 0.5 * h) / (0.5 * h);
      out.headingErr = std::clamp((farCx - nearCx) / halfW, -1.0, 1.0);

      const int bw = std::max(1, maxX - minX + 1);
      const int bh = std::max(1, maxY - minY + 1);
      const double elong = static_cast<double>(std::max(bw, bh)) /
                           static_cast<double>(std::max(1, std::min(bw, bh)));

      const double n = static_cast<double>(darkCount);
      const double mu20 = (sxx / n) - (cx * cx);
      const double mu02 = (syy / n) - (cy * cy);
      const double mu11 = (sxy / n) - (cx * cy);
      out.lineAngle = 0.5 * std::atan2(2.0 * mu11, (mu20 - mu02));

      out.area = static_cast<double>(darkCount) / (w * h);

      // Relaxed detection logic for 2m+ distances
      out.cableDetected = (out.area > this->visionAreaMin && 
                           out.area < this->visionAreaMax && 
                           elong > 1.01);

      static int debug_print_count = 0;
      if (++debug_print_count % 50 == 0) {
        std::cout << "[VisionData] dark=" << darkCount << " area=" << out.area 
                  << " elong=" << elong << " det=" << (out.cableDetected ? 1 : 0) 
                  << " xErr=" << out.xErr << " headErr=" << out.headingErr << std::endl;
      }
    }
    else {
      static int empty_print_count = 0;
      if (++empty_print_count % 50 == 0) {
        std::cout << "[VisionData] NO DARK PIXELS FOUND (darkCount=" << darkCount << ")" << std::endl;
      }
    }

    std::lock_guard<std::mutex> lock(this->visionMtx);
    this->vision = out;
  }

  private: void BuildCableMap(EntityComponentManager &_ecm)
  {
    this->cablePoints.clear();

    Entity cableModel = kNullEntity;

    // Find model named submarine_cable
    _ecm.Each<components::Name, components::Pose>(
      [&](const Entity &_e, const components::Name *_name, const components::Pose *) -> bool
      {
        if (_name && _name->Data() == this->cableModelName)
        {
          cableModel = _e;
          return false;
        }
        return true;
      });

    if (cableModel == kNullEntity)
      return;

    std::map<int, std::pair<bool, math::Color>> visualInfo;
    std::map<int, Entity> visualEntities;

    // Gather red/non-red status from visual materials.
    _ecm.Each<components::Name, components::Visual, components::Material>(
      [&](const Entity &_e, const components::Name *_name,
          const components::Visual *, const components::Material *_mat) -> bool
      {
        if (!_name || !_mat)
          return true;

        const std::string n = _name->Data();
        const std::string prefix = "segment_visual_";
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

        const sdf::Material &mat = _mat->Data();
        const bool isRed = this->IsMaterialRed(mat);
        math::Color color = mat.Diffuse();
        if (color.R() <= 0.001 && color.G() <= 0.001 && color.B() <= 0.001)
          color = mat.Ambient();
        visualInfo[idx] = {isRed, color};
        visualEntities[idx] = _e;
        return true;
      });

    // Gather segment collision world poses and geometry.
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

        const math::Pose3d wp = this->WorldPose(_e, _ecm);
        CablePoint cp;
        cp.idx = idx;
        cp.collisionEntity = _e;
        cp.pos = wp.Pos();
        cp.length = geom.CylinderShape()->Length();
        cp.radius = geom.CylinderShape()->Radius();
        cp.axis = wp.Rot().RotateVector(math::Vector3d::UnitZ);
        if (cp.axis.Length() > 1e-6)
          cp.axis.Normalize();
        if (visualInfo.count(idx) > 0)
        {
          cp.isRed = visualInfo[idx].first;
          cp.color = visualInfo[idx].second;
        }
        if (visualEntities.count(idx) > 0)
          cp.visualEntity = visualEntities[idx];
        cp.isRed = cp.isRed || (this->corrosionSegments.count(idx) > 0);
        this->cablePoints.push_back(cp);
        return true;
      });

    std::sort(this->cablePoints.begin(), this->cablePoints.end(),
      [](const CablePoint &a, const CablePoint &b){ return a.idx < b.idx; });

    this->UpdateStationBStopPoint(_ecm);
  }

  private: void NearestCablePoint(
      const math::Vector3d &_robotPos,
      int &_nearestIdx,
      math::Vector3d &_nearestPos,
      double &_nearestDist) const
  {
    _nearestIdx = -1;
    _nearestDist = 1e9;

    for (const auto &cp : this->cablePoints)
    {
      const double d = (_robotPos - cp.pos).Length();
      if (d < _nearestDist)
      {
        _nearestDist = d;
        _nearestIdx = cp.idx;
        _nearestPos = cp.pos;
      }
    }
  }

  private: math::Vector3d CableTangent(int _idx) const
  {
    if (this->cablePoints.size() < 2 || _idx < 0)
      return math::Vector3d(1, 0, 0);

    int i = -1;
    for (size_t k = 0; k < this->cablePoints.size(); ++k)
    {
      if (this->cablePoints[k].idx == _idx)
      {
        i = static_cast<int>(k);
        break;
      }
    }

    if (i < 0)
      return math::Vector3d(1, 0, 0);

    math::Vector3d t;
    if (i == 0)
      t = this->cablePoints[1].pos - this->cablePoints[0].pos;
    else if (i == static_cast<int>(this->cablePoints.size()) - 1)
      t = this->cablePoints[i].pos - this->cablePoints[i - 1].pos;
    else
      t = this->cablePoints[i + 1].pos - this->cablePoints[i - 1].pos;

    if (t.Length() < 1e-6)
      return math::Vector3d(1, 0, 0);

    t.Normalize();
    return t;
  }

  private: int FindCablePointVectorIndex(int _idx) const
  {
    for (size_t i = 0; i < this->cablePoints.size(); ++i)
    {
      if (this->cablePoints[i].idx == _idx)
        return static_cast<int>(i);
    }
    return -1;
  }

  private: bool IsMaterialRed(const sdf::Material &_mat) const
  {
    const auto d = _mat.Diffuse();
    const auto a = _mat.Ambient();
    const bool diffRed = d.R() > 0.2 && d.R() > d.G() * 2.0 && d.R() > d.B() * 2.0;
    const bool ambRed = a.R() > 0.2 && a.R() > a.G() * 2.0 && a.R() > a.B() * 2.0;
    return diffRed || ambRed;
  }

  private: GapMeasure MeasureSegmentGap(const CablePoint &_a, const CablePoint &_b) const
  {
    const math::Vector3d a1 = _a.pos + _a.axis * (_a.length * 0.5);
    const math::Vector3d a2 = _a.pos - _a.axis * (_a.length * 0.5);
    const math::Vector3d b1 = _b.pos + _b.axis * (_b.length * 0.5);
    const math::Vector3d b2 = _b.pos - _b.axis * (_b.length * 0.5);

    const std::array<std::pair<math::Vector3d, math::Vector3d>, 4> pairs = {{
      {a1, b1}, {a1, b2}, {a2, b1}, {a2, b2}
    }};

    GapMeasure out;
    out.dist = 1e9;
    for (const auto &p : pairs)
    {
      const double d = (p.first - p.second).Length();
      if (d < out.dist)
      {
        out.dist = d;
        out.endA = p.first;
        out.endB = p.second;
        out.mid = (p.first + p.second) * 0.5;
      }
    }
    return out;
  }

  private: std::string GapKey(int _idxA, int _idxB) const
  {
    const int a = std::min(_idxA, _idxB);
    const int b = std::max(_idxA, _idxB);
    return std::to_string(a) + ":" + std::to_string(b);
  }

  private: bool RepairCableBreak(const FaultInfo &_fault)
  {
    if (_fault.type != "cable_break" || _fault.idxA < 0 || _fault.idxB < 0)
      return false;

    const std::string key = this->GapKey(_fault.idxA, _fault.idxB);
    if (this->repairedGaps.count(key) > 0)
      return true;

    if (this->worldName.empty())
      return false;

    const math::Vector3d axis = _fault.endB - _fault.endA;
    const double len = axis.Length();
    if (len < 1e-4)
      return false;

    math::Quaterniond q;
    q.SetFrom2Axes(math::Vector3d::UnitZ, axis.Normalized());
    const auto rpy = q.Euler();

    const int patchIdx = _fault.idxA + 1;
    const std::string patchModelName = "repair_patch_segment_" + std::to_string(this->patchCounter++);

    std::ostringstream sdf;
    sdf << std::fixed << std::setprecision(6);
    sdf << "<sdf version='1.9'>"
        << "<model name='" << patchModelName << "'>"
        << "<static>true</static>"
        << "<pose>0 0 0 0 0 0</pose>"
        << "<link name='link'>"
        << "<collision name='segment_collision_" << patchIdx << "'>"
        << "<geometry><cylinder><radius>" << _fault.radius
        << "</radius><length>" << len
        << "</length></cylinder></geometry>"
        << "</collision>"
        << "<visual name='segment_visual_" << patchIdx << "'>"
        << "<geometry><cylinder><radius>" << _fault.radius
        << "</radius><length>" << len
        << "</length></cylinder></geometry>"
        << "<material><ambient>"
        << _fault.color.R() << " " << _fault.color.G() << " " << _fault.color.B() << " 1</ambient>"
        << "<diffuse>"
        << _fault.color.R() << " " << _fault.color.G() << " " << _fault.color.B() << " 1</diffuse>"
        << "</material>"
        << "</visual>"
        << "</link>"
        << "</model>"
        << "</sdf>";

    msgs::EntityFactory req;
    req.set_sdf(sdf.str());
    req.set_allow_renaming(true);
    auto *pose = req.mutable_pose();
    pose->mutable_position()->set_x(_fault.pos.X());
    pose->mutable_position()->set_y(_fault.pos.Y());
    pose->mutable_position()->set_z(_fault.pos.Z());
    pose->mutable_orientation()->set_w(q.W());
    pose->mutable_orientation()->set_x(q.X());
    pose->mutable_orientation()->set_y(q.Y());
    pose->mutable_orientation()->set_z(q.Z());

    std::vector<std::string> worldsToTry;
    if (!this->worldName.empty())
      worldsToTry.push_back(this->worldName);
    for (const std::string &fallback : {"default", "ocean_world", "underwater_world"})
    {
      if (std::find(worldsToTry.begin(), worldsToTry.end(), fallback) == worldsToTry.end())
        worldsToTry.push_back(fallback);
    }

    bool spawnOk = false;
    std::ostringstream failReason;
    for (const auto &world : worldsToTry)
    {
      const std::string createSrv = "/world/" + world + "/create";
      msgs::Boolean rep;
      bool result = false;
      const bool executed = this->node.Request(createSrv, req, 3000u, rep, result);
      if (executed && result && rep.data())
      {
        this->worldName = world;
        spawnOk = true;
        break;
      }

      failReason << "[" << createSrv
                 << " executed=" << executed
                 << " result=" << result
                 << " data=" << rep.data() << "] ";
    }

    if (!spawnOk)
    {
      gzwarn << "[Repair] Spawn failed. Tried: " << failReason.str() << "\n";
      return false;
    }

    this->repairedGaps.insert(key);
    gzwarn << "CABLE RECONNECTED\n";
    std::cout << "CABLE RECONNECTED\n" << std::flush;
    return true;
  }

  private: bool RepairCorrosion(const FaultInfo &_fault, EntityComponentManager &_ecm)
  {
    if (_fault.type != "corrosion" || _fault.visualEntity == kNullEntity)
      return false;

    auto *matComp = _ecm.Component<components::Material>(_fault.visualEntity);
    if (!matComp)
    {
      _ecm.CreateComponent(_fault.visualEntity, components::Material());
      matComp = _ecm.Component<components::Material>(_fault.visualEntity);
    }
    if (!matComp)
      return false;

    sdf::Material mat = matComp->Data();
    const math::Color black(0.01f, 0.01f, 0.01f, 1.0f);
    mat.SetAmbient(black);
    mat.SetDiffuse(black);
    mat.SetSpecular(math::Color(0, 0, 0, 1));
    mat.SetEmissive(math::Color(0, 0, 0, 1));
    matComp->SetData(mat, [](const sdf::Material &a, const sdf::Material &b)
      {
        return a.Diffuse() == b.Diffuse() &&
               a.Ambient() == b.Ambient() &&
               a.Specular() == b.Specular() &&
               a.Emissive() == b.Emissive();
      });

    _ecm.SetChanged(_fault.visualEntity, components::Material::typeId, ComponentState::OneTimeChange);

    // Also send a VisualCmd material update so renderers apply the change
    // immediately.
    auto *visCmdComp = _ecm.Component<components::VisualCmd>(_fault.visualEntity);
    if (!visCmdComp)
    {
      _ecm.CreateComponent(_fault.visualEntity, components::VisualCmd());
      visCmdComp = _ecm.Component<components::VisualCmd>(_fault.visualEntity);
    }
    if (visCmdComp)
    {
      auto &cmd = visCmdComp->Data();
      auto *msgMat = cmd.mutable_material();
      msgMat->mutable_ambient()->set_r(black.R());
      msgMat->mutable_ambient()->set_g(black.G());
      msgMat->mutable_ambient()->set_b(black.B());
      msgMat->mutable_ambient()->set_a(black.A());
      msgMat->mutable_diffuse()->set_r(black.R());
      msgMat->mutable_diffuse()->set_g(black.G());
      msgMat->mutable_diffuse()->set_b(black.B());
      msgMat->mutable_diffuse()->set_a(black.A());
      msgMat->mutable_specular()->set_r(0.0);
      msgMat->mutable_specular()->set_g(0.0);
      msgMat->mutable_specular()->set_b(0.0);
      msgMat->mutable_specular()->set_a(1.0);
      msgMat->mutable_emissive()->set_r(0.0);
      msgMat->mutable_emissive()->set_g(0.0);
      msgMat->mutable_emissive()->set_b(0.0);
      msgMat->mutable_emissive()->set_a(1.0);
      _ecm.SetChanged(_fault.visualEntity, components::VisualCmd::typeId,
                      ComponentState::OneTimeChange);
    }

    this->corrosionSegments.erase(_fault.idx);

    gzwarn << "CORROSION REPAIRED\n";
    std::cout << "CORROSION REPAIRED\n" << std::flush;
    return true;
  }

  private: void DiscoverWorldName(EntityComponentManager &_ecm)
  {
    _ecm.Each<components::World, components::Name>(
      [&](const Entity &, const components::World *, const components::Name *_name) -> bool
      {
        if (_name)
        {
          this->worldName = _name->Data();
          return false;
        }
        return true;
      });
  }

  private: std::optional<FaultInfo> DetectFaultUnderRobot(const math::Vector3d &_robotPos) const
  {
    if (this->cablePoints.empty())
      return std::nullopt;

    int nearestVecIdx = -1;
    double nearestXY = 1e9;
    for (size_t i = 0; i < this->cablePoints.size(); ++i)
    {
      const auto &cp = this->cablePoints[i];
      const math::Vector3d cpPos = this->CableSegmentPosition(cp);
      const double dxy = std::hypot(cpPos.X() - _robotPos.X(), cpPos.Y() - _robotPos.Y());
      if (dxy < nearestXY)
      {
        nearestXY = dxy;
        nearestVecIdx = static_cast<int>(i);
      }
    }

    if (nearestVecIdx < 0 || nearestXY > this->inspectionRadius)
      return std::nullopt;

    const CablePoint &cp = this->cablePoints[nearestVecIdx];
    if (cp.isRed && this->reportedCorrosionSegments.count(cp.idx) == 0)
    {
      FaultInfo out;
      out.type = "CORROSION";
      this->SetFaultCoordinates(out, this->CableSegmentPosition(cp));
      out.idx = cp.idx;
      out.visualEntity = cp.visualEntity;
      return out;
    }

    // Break fault: gap between adjacent segment centers in the XY plane > 0.5 m.
    for (int i : {nearestVecIdx - 1, nearestVecIdx})
    {
      if (i < 0 || i + 1 >= static_cast<int>(this->cablePoints.size()))
        continue;

      const int idxA = this->cablePoints[i].idx;
      const int idxB = this->cablePoints[i + 1].idx;
      const std::string gapKey = this->GapKey(idxA, idxB);
      if (this->repairedGaps.count(gapKey) > 0 || this->reportedBreakGaps.count(gapKey) > 0)
        continue;

      const math::Vector3d posA = this->CableSegmentPosition(this->cablePoints[i]);
      const math::Vector3d posB = this->CableSegmentPosition(this->cablePoints[i + 1]);
      const double dx = posB.X() - posA.X();
      const double dy = posB.Y() - posA.Y();
      const double distance = std::sqrt((dx * dx) + (dy * dy));
      const math::Vector3d breakPos = (posA + posB) * 0.5;
      const double dxy = std::hypot(breakPos.X() - _robotPos.X(), breakPos.Y() - _robotPos.Y());
      if (distance > 0.5 && dxy <= this->inspectionRadius)
      {
        FaultInfo out;
        out.type = "CABLE_BREAK";
        this->SetFaultCoordinates(out, breakPos);
        out.idxA = idxA;
        out.idxB = idxB;
        return out;
      }
    }

    return std::nullopt;
  }

  private: void SetFaultCoordinates(FaultInfo &_fault, const math::Vector3d &_pos) const
  {
    _fault.pos = _pos;
    _fault.x = _pos.X();
    _fault.y = _pos.Y();
    _fault.z = _pos.Z();
    _fault.depth = _pos.Z();
  }

  private: void StoreFault(const FaultInfo &_fault)
  {
    this->detectedFaults.push_back(_fault);
  }

  private: void UpdateStopMarker(EntityComponentManager &_ecm)
  {
    if (this->stopMarkerEntity == kNullEntity)
    {
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

    if (this->stopMarkerEntity != kNullEntity)
    {
      this->stopMarkerPos = this->WorldPose(this->stopMarkerEntity, _ecm).Pos();
      this->stopMarkerValid = true;
    }
  }

  private: bool ReachedStopMarker(const math::Vector3d &_robotPos) const
  {
    if (!this->stopMarkerValid)
      return false;

    return (_robotPos - this->stopMarkerPos).Length() < this->stopMarkerReachDistance;
  }

  private: void LockRobotStatic(EntityComponentManager &_ecm)
  {
    if (this->inspectionStaticApplied)
      return;

    _ecm.RemoveComponent<components::Static>(this->model.Entity());
    _ecm.CreateComponent(this->model.Entity(), components::Static(true));
    this->inspectionStaticApplied = true;
  }

  private: void UpdateStationBStopPoint(EntityComponentManager &_ecm)
  {
    math::Vector3d stationReference(this->stationB_x, this->stationB_y, this->stationB_z);
    Entity stationEntity = kNullEntity;

    _ecm.Each<components::Name, components::Pose>(
      [&](const Entity &_e, const components::Name *_name, const components::Pose *) -> bool
      {
        if (_name && _name->Data() == this->stationBModelName)
        {
          stationEntity = _e;
          return false;
        }
        return true;
      });

    if (stationEntity != kNullEntity)
      stationReference = this->WorldPose(stationEntity, _ecm).Pos();

    this->stationB_x = stationReference.X();
    this->stationB_y = stationReference.Y();
    this->stationB_z = stationReference.Z();

    if (this->cablePoints.empty())
    {
      this->stationBStopPoint = stationReference;
      this->stationBStopPointValid = true;
      return;
    }

    const double distToFirst =
      (this->cablePoints.front().pos - stationReference).Length();
    const double distToLast =
      (this->cablePoints.back().pos - stationReference).Length();
    const bool stationAtHighIndexEnd = distToLast <= distToFirst;

    int corrosionVecIndex = -1;
    if (stationAtHighIndexEnd)
    {
      for (int i = static_cast<int>(this->cablePoints.size()) - 1; i >= 0; --i)
      {
        if (this->cablePoints[i].isRed)
        {
          corrosionVecIndex = i;
          break;
        }
      }
    }
    else
    {
      for (size_t i = 0; i < this->cablePoints.size(); ++i)
      {
        if (this->cablePoints[i].isRed)
        {
          corrosionVecIndex = static_cast<int>(i);
          break;
        }
      }
    }

    int stopPointIndex = 0;
    if (corrosionVecIndex >= 0)
    {
      stopPointIndex = corrosionVecIndex +
        (stationAtHighIndexEnd
          ? this->stationBStopSegmentsAfterCorrosion
          : -this->stationBStopSegmentsAfterCorrosion);
    }
    else if (stationAtHighIndexEnd)
    {
      stopPointIndex = static_cast<int>(this->cablePoints.size()) - 1;
    }
    else
    {
      stopPointIndex = 0;
    }

    stopPointIndex = std::clamp(
      stopPointIndex,
      0,
      static_cast<int>(this->cablePoints.size()) - 1);
    this->stationBStopSegmentIdx = this->cablePoints[stopPointIndex].idx;
    this->stationBAtHighIndexEnd = stationAtHighIndexEnd;
    this->stationBStopPoint = this->cablePoints[stopPointIndex].pos;
    this->stationBStopPointValid = true;
  }

  private: bool ReachedStationB(const math::Vector3d &_robotPos, int _nearestIdx) const
  {
    const math::Vector3d stopPoint = this->stationBStopPointValid
      ? this->stationBStopPoint
      : math::Vector3d(this->stationB_x, this->stationB_y, this->stationB_z);
    const double dx = _robotPos.X() - stopPoint.X();
    const double dy = _robotPos.Y() - stopPoint.Y();
    const double distance = std::sqrt((dx * dx) + (dy * dy));
    if (distance < this->stationBReachDistance)
      return true;

    if (_nearestIdx < 0 || this->stationBStopSegmentIdx < 0)
      return false;

    if (this->stationBAtHighIndexEnd)
      return _nearestIdx >=
        (this->stationBStopSegmentIdx - this->stationBStopSegmentSlack);

    return _nearestIdx <=
      (this->stationBStopSegmentIdx + this->stationBStopSegmentSlack);
  }

  private: void PrintStopMarkerReached() const
  {
    const std::string message = "[CableStateController] Robot stopped at marker location\n";
    gzmsg << message;
    std::cout << message << std::flush;
  }

  private: void PrintInspectionReport() const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "====================================" << std::endl;
    out << "CABLE INSPECTION REPORT" << std::endl;
    out << "====================================" << std::endl;
    out << std::endl;

    for (size_t i = 0; i < this->detectedFaults.size(); ++i)
    {
      const auto &fault = this->detectedFaults[i];
      out << "Fault " << (i + 1) << std::endl;
      out << "Type: " << fault.type << std::endl;
      out << "Coordinates: (" << fault.x << ", " << fault.y << ", " << fault.z << ")" << std::endl;
      out << "Depth: " << fault.depth << " meters" << std::endl;
      out << std::endl;
    }

    out << "------------------------------------" << std::endl;
    out << "Total Faults Detected: " << this->detectedFaults.size() << std::endl;
    out << "Inspection Status: COMPLETED" << std::endl;
    out << "Robot stopped at marker location" << std::endl;
    out << "====================================" << std::endl;
    const std::string report = out.str();
    gzmsg << report;
    std::cout << report << std::flush;
  }

  private: void MarkFaultReported(const FaultInfo &_fault)
  {
    if (_fault.type == "CORROSION" && _fault.idx >= 0)
    {
      this->reportedCorrosionSegments.insert(_fault.idx);
      return;
    }

    if (_fault.type == "CABLE_BREAK" && _fault.idxA >= 0 && _fault.idxB >= 0)
      this->reportedBreakGaps.insert(this->GapKey(_fault.idxA, _fault.idxB));
  }

  private: math::Pose3d WorldPose(
      const Entity &_entity,
      EntityComponentManager &_ecm) const
  {
    if (_entity == kNullEntity)
      return math::Pose3d();

    return worldPose(_entity, _ecm);
  }

  private: math::Pose3d WorldPose(const Entity &_entity) const
  {
    if (_entity == kNullEntity || this->activeEcm == nullptr)
      return math::Pose3d();

    return this->WorldPose(_entity, *this->activeEcm);
  }

  private: math::Vector3d CableSegmentPosition(const CablePoint &_cp) const
  {
    if (_cp.collisionEntity != kNullEntity && this->activeEcm != nullptr)
      return this->WorldPose(_cp.collisionEntity).Pos();

    return _cp.pos;
  }

  private: void PublishCmd(double _lx, double _ly, double _lz,
                           double _ax, double _ay, double _az)
  {
    if (this->publishVelocityTopic)
    {
      msgs::Twist cmd;
      cmd.mutable_linear()->set_x(_lx);
      cmd.mutable_linear()->set_y(_ly);
      cmd.mutable_linear()->set_z(_lz);
      cmd.mutable_angular()->set_x(_ax);
      cmd.mutable_angular()->set_y(_ay);
      cmd.mutable_angular()->set_z(_az);
      this->cmdPub.Publish(cmd);
    }

    if (this->activeEcm == nullptr || this->baseLink == kNullEntity)
      return;

    // Ensure direct velocity command components are not latched; this lets
    // hydrodynamic forces and ocean current influence motion.
    this->activeEcm->RemoveComponent<components::LinearVelocityCmd>(this->baseLink);
    this->activeEcm->RemoveComponent<components::AngularVelocityCmd>(this->baseLink);
    this->activeEcm->RemoveComponent<components::LinearVelocityCmd>(this->model.Entity());
    this->activeEcm->RemoveComponent<components::AngularVelocityCmd>(this->model.Entity());

    Link link(this->baseLink);
    link.EnableVelocityChecks(*this->activeEcm, true);
    const auto linVelOpt = link.WorldLinearVelocity(*this->activeEcm);
    const auto angVelOpt = link.WorldAngularVelocity(*this->activeEcm);
    if (!linVelOpt.has_value() || !angVelOpt.has_value())
      return;

    const math::Pose3d basePose = worldPose(this->baseLink, *this->activeEcm);
    const math::Vector3d desLinWorld = basePose.Rot().RotateVector(math::Vector3d(_lx, _ly, _lz));
    const math::Vector3d desAngWorld = basePose.Rot().RotateVector(math::Vector3d(_ax, _ay, _az));

    const math::Vector3d linErr = desLinWorld - *linVelOpt;
    const math::Vector3d angErr = desAngWorld - *angVelOpt;

    double mass = 55.0;
    if (const auto *inertial = this->activeEcm->Component<components::Inertial>(this->baseLink))
      mass = std::max(1.0, inertial->Data().MassMatrix().Mass());

    math::Vector3d force = linErr * (this->forceVelGain * mass);
    if (force.Length() > this->maxForce)
      force = force.Normalized() * this->maxForce;

    math::Vector3d torque = angErr * this->torqueAngGain;
    if (torque.Length() > this->maxTorque)
      torque = torque.Normalized() * this->maxTorque;

    link.AddWorldForce(*this->activeEcm, force);
    link.AddWorldWrench(*this->activeEcm, math::Vector3d::Zero, torque);
  }

  private: void PublishStop()
  {
    this->PublishCmd(0, 0, 0, 0, 0, 0);
  }

  private: void Transition(State _next)
  {
    if (this->state == _next)
      return;
    this->state = _next;
    this->stateEnter = std::chrono::steady_clock::now();
  }

  private: static double NormalizeAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  private: const char *StateName(State _s) const
  {
    switch (_s)
    {
      case State::SEARCH_CABLE: return "SEARCH_CABLE";
      case State::STOP_AT_CABLE: return "STOP_AT_CABLE";
      case State::ALIGN_WITH_CABLE: return "ALIGN_WITH_CABLE";
      case State::FOLLOW_CABLE: return "FOLLOW_CABLE";
      case State::HALTED: return "HALTED";
    }
    return "UNKNOWN";
  }

  private: void ParseCorrosionSegments(const std::string &_csv)
  {
    this->corrosionSegments.clear();
    std::stringstream ss(_csv);
    std::string token;
    while (std::getline(ss, token, ','))
    {
      try
      {
        const int id = std::stoi(token);
        this->corrosionSegments.insert(id);
      }
      catch (...) {}
    }
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

  private: bool GetSdfB(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      bool _def) const
  {
    if (_sdf && _sdf->HasElement(_name))
      return _sdf->Get<bool>(_name);
    return _def;
  }

  private: bool VisionConfident(const Vision &_v) const
  {
    return _v.valid && _v.cableDetected &&
           _v.area >= this->visionAreaMin &&
           _v.area <= this->visionAreaMax;
  }

  private: bool VisionFresh(const std::chrono::steady_clock::duration &_simTime) const
  {
    if (this->lastVisionSeenSimTime == std::chrono::steady_clock::duration::zero())
      return false;
    const double dt = std::chrono::duration<double>(_simTime - this->lastVisionSeenSimTime).count();
    return dt <= this->maxVisionLossSec;
  }

  private: double VisionSteerCmd(
      const Vision &_v,
      double _crossGain,
      double _headingGain,
      double _limit) const
  {
    const double steer = -(_crossGain * _v.xErr + _headingGain * _v.headingErr);
    return std::clamp(steer, -_limit, _limit);
  }

  private: Model model{kNullEntity};
  private: Entity baseLink{kNullEntity};
  private: EntityComponentManager *activeEcm{nullptr};
  private: transport::Node node;
  private: transport::Node::Publisher cmdPub;

  private: std::mutex visionMtx;
  private: Vision vision;

  private: bool configured{false};
  private: State state{State::SEARCH_CABLE};
  private: std::chrono::steady_clock::time_point stateEnter{};

  private: std::string robotName{"cable_repair_robot"};
  private: std::string cableModelName{"submarine_cable"};
  private: std::string cameraTopic{"/cable_repair_robot/camera"};
  private: std::string cmdTopic{"/model/cable_repair_robot/LinearVelocityCmd"};
  private: std::string worldName{""};

  private: double targetZ{0.85};
  private: double stopDistance{0.5};
  private: double stopAtCableSec{0.6};
  private: double detectDistance{1.2};
  private: double minSearchTime{5.0};
  private: double faultStopSec{0.8};
  private: double maxVisionLossSec{0.8};
  private: double visionAreaMin{0.00008};
  private: double visionAreaMax{0.25};
  private: double visionFallbackLockDistance{0.45};
  private: double visionDarkLumThreshold{75.0};
  private: bool visionRequiredForLock{true};
  private: bool visionRequiredForRepair{true};
  private: bool publishVelocityTopic{false};
  private: double forceVelGain{9.0};
  private: double torqueAngGain{7.0};
  private: double maxForce{180.0};
  private: double maxTorque{65.0};
  private: double cableMapRefreshSec{0.2};
  private: double inspectionRadius{0.7};
  private: std::string stopMarkerName{"stop_marker"};
  private: Entity stopMarkerEntity{kNullEntity};
  private: math::Vector3d stopMarkerPos{38.7970, -0.1226, 3.0};
  private: bool stopMarkerValid{false};
  private: double stopMarkerReachDistance{1.0};
  private: std::string stationBModelName{"station_country_B"};
  private: double stationB_x{40.9};
  private: double stationB_y{-0.1};
  private: double stationB_z{2.9};
  private: math::Vector3d stationBStopPoint{38.7970, -0.1226, 3.0};
  private: bool stationBStopPointValid{false};
  private: int stationBStopSegmentIdx{-1};
  private: bool stationBAtHighIndexEnd{true};
  private: int stationBStopSegmentsAfterCorrosion{1};
  private: int stationBStopSegmentSlack{1};
  private: double stationBReachDistance{0.75};
  private: double lostCableDistance{1.8};
  private: int followDirection{1};
  private: int turnaroundsDone{0};
  private: int maxTurnarounds{1};
  private: int patchCounter{1};
  private: bool inspectionCompleted{false};
  private: bool inspectionReportPrinted{false};
  private: bool inspectionStaticApplied{false};
  private: std::chrono::steady_clock::duration faultStopUntil{std::chrono::steady_clock::duration::zero()};
  private: std::chrono::steady_clock::duration lastDebugPrint{std::chrono::steady_clock::duration::zero()};
  private: std::chrono::steady_clock::duration lastAlignPrint{std::chrono::steady_clock::duration::zero()};
  private: std::chrono::steady_clock::duration lastDetectionLogPrint{std::chrono::steady_clock::duration::zero()};
  private: std::chrono::steady_clock::duration lastVisionSeenSimTime{std::chrono::steady_clock::duration::zero()};
  private: std::chrono::steady_clock::duration lastCableMapRefreshSimTime{std::chrono::steady_clock::duration::zero()};

  private: std::vector<CablePoint> cablePoints;
  private: std::vector<FaultInfo> detectedFaults;
  private: std::set<int> corrosionSegments;
  private: std::set<std::string> repairedGaps;
  private: std::set<int> reportedCorrosionSegments;
  private: std::set<std::string> reportedBreakGaps;
  private: std::map<std::string, std::chrono::steady_clock::duration> repairRetryAfter;
  private: double repairRetrySec{2.0};
};

}  // namespace cable_robot_controller

GZ_ADD_PLUGIN(
  cable_robot_controller::CableStateController,
  gz::sim::System,
  cable_robot_controller::CableStateController::ISystemConfigure,
  cable_robot_controller::CableStateController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(cable_robot_controller::CableStateController, "cable_robot_controller")
GZ_ADD_PLUGIN_ALIAS(cable_robot_controller::CableStateController, "cable_robot_controller::CableStateController")
