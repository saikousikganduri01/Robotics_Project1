#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <regex>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace
{
constexpr double kMinDt = 1e-6;
constexpr double kEps = 1e-9;
constexpr double kTwoPi = 6.28318530717958647692;
}

namespace ocean_current_plugin
{

using namespace gz;
using namespace sim;

class CurrentSystem:
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
  struct LinkState
  {
    Entity parentModel{kNullEntity};
    std::string name;
    double mass{30.0};
    bool staticModel{false};
  };

  struct SegmentPoint
  {
    math::Vector3d localPos{0, 0, 0};
    std::size_t index{0};
  };

  public: void Configure(const Entity &,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &) override
  {
    this->modelRegex = std::regex(this->GetString(_sdf, "model_name_regex", ".*"));
    this->cableModelName = this->GetString(_sdf, "cable_model_name", "submarine_cable");
    this->segmentRegex = std::regex(this->GetString(_sdf, "cable_segment_collision_regex", "segment_collision_.*"));

    this->minSpeed = this->GetDouble(_sdf, "min_current_speed", 0.1);
    this->maxSpeed = this->GetDouble(_sdf, "max_current_speed", 0.5);
    if (this->minSpeed > this->maxSpeed)
      std::swap(this->minSpeed, this->maxSpeed);

    this->speedTau = std::max(1.0, this->GetDouble(_sdf, "speed_time_constant", 55.0));
    this->headingTau = std::max(1.0, this->GetDouble(_sdf, "heading_time_constant", 70.0));
    this->speedNoise = std::max(0.0, this->GetDouble(_sdf, "speed_noise_stddev", 0.02));
    this->headingNoise = std::max(0.0, this->GetDouble(_sdf, "heading_noise_stddev", 0.03));
    this->verticalNoise = std::max(0.0, this->GetDouble(_sdf, "vertical_noise_stddev", 0.004));

    this->linearDragCoef = std::max(0.0, this->GetDouble(_sdf, "linear_drag_coefficient", 0.42));
    this->maxAccel = std::max(0.01, this->GetDouble(_sdf, "max_accel", 0.35));
    this->cableLinearDragScale = std::clamp(
      this->GetDouble(_sdf, "cable_linear_drag_scale", 1.0), 0.0, 1.0);

    this->cableForceScale = std::max(0.0, this->GetDouble(_sdf, "cable_force_scale", 0.22));
    this->cableSwayAmplitude = std::max(0.0, this->GetDouble(_sdf, "cable_sway_amplitude", 0.03));
    this->cableSwayFrequency = std::max(0.05, this->GetDouble(_sdf, "cable_sway_frequency", 0.09));
    this->cableSwayDamping = std::max(0.1, this->GetDouble(_sdf, "cable_sway_damping", 1.6));
    this->cableMaxForcePerSegment = std::max(0.01, this->GetDouble(_sdf, "cable_max_force_per_segment", 2.2));
    this->cableAnchorLinearStiffness = std::max(
      0.0, this->GetDouble(_sdf, "cable_anchor_linear_stiffness", 0.0));
    this->cableAnchorLinearDamping = std::max(
      0.0, this->GetDouble(_sdf, "cable_anchor_linear_damping", 0.0));
    this->cableAnchorAngularStiffness = std::max(
      0.0, this->GetDouble(_sdf, "cable_anchor_angular_stiffness", 0.0));
    this->cableAnchorAngularDamping = std::max(
      0.0, this->GetDouble(_sdf, "cable_anchor_angular_damping", 0.0));
    this->cableAnchorMaxForce = std::max(
      0.0, this->GetDouble(_sdf, "cable_anchor_max_force", 1e9));
    this->cableAnchorMaxTorque = std::max(
      0.0, this->GetDouble(_sdf, "cable_anchor_max_torque", 1e9));

    const double initialSpeed = std::clamp(this->GetDouble(_sdf, "initial_current_speed", 0.24), this->minSpeed, this->maxSpeed);
    const double initialHeading = this->GetDouble(_sdf, "initial_heading_rad", 0.35);
    this->currentSpeed = initialSpeed;
    this->currentHeading = initialHeading;
    this->meanCurrent = math::Vector3d(std::cos(initialHeading) * initialSpeed,
                                       std::sin(initialHeading) * initialSpeed,
                                       0.0);

    const int seed = this->GetInt(_sdf, "random_seed", 7);
    this->rng.seed(seed);

    this->node.Subscribe("/ocean_current", &CurrentSystem::OnCurrentCmd, this);
    this->pub = this->node.Advertise<msgs::Vector3d>("/ocean_current");

    this->discoverPeriod = std::chrono::milliseconds(static_cast<int>(
      std::max(250.0, this->GetDouble(_sdf, "discover_period_ms", 1500.0))));
    this->publishPeriod = std::chrono::milliseconds(static_cast<int>(
      std::max(100.0, this->GetDouble(_sdf, "publish_period_ms", 300.0))));

    this->DiscoverLinks(_ecm, true);
    this->DiscoverCableSegments(_ecm, true);
    this->CacheCableReferencePose(_ecm);

    gzmsg << "[CurrentSystem] configured. speed_range=[" << this->minSpeed << ", "
          << this->maxSpeed << "] m/s, cableSegments="
          << this->cableSegmentLocalPoints.size() << "\n";
  }

  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    const auto dtSec = std::max(kMinDt,
      std::chrono::duration<double>(_info.dt).count());

    if (this->lastDiscoverSimTime == std::chrono::steady_clock::duration::zero() ||
        _info.simTime - this->lastDiscoverSimTime >= this->discoverPeriod)
    {
      this->lastDiscoverSimTime = _info.simTime;
      this->DiscoverLinks(_ecm, false);
      this->DiscoverCableSegments(_ecm, false);
      this->CacheCableReferencePose(_ecm);
    }

    this->UpdateCurrent(dtSec, _info.simTime);
    this->ApplyHydrodynamicForces(_ecm);
    this->ApplyCableSway(_ecm, dtSec, _info.simTime);
    this->ApplyCableAnchor(_ecm);

    if (this->lastPublishSimTime == std::chrono::steady_clock::duration::zero() ||
        _info.simTime - this->lastPublishSimTime >= this->publishPeriod)
    {
      this->lastPublishSimTime = _info.simTime;
      this->PublishCurrent();
    }
  }

  private: void OnCurrentCmd(const msgs::Vector3d &_msg)
  {
    const math::Vector3d requested(_msg.x(), _msg.y(), _msg.z());
    const double horizontalSpeed = std::hypot(requested.X(), requested.Y());
    if (horizontalSpeed < kEps)
      return;

    const math::Vector3d dir = requested / std::max(horizontalSpeed, kEps);
    const double clampedSpeed = std::clamp(horizontalSpeed, this->minSpeed, this->maxSpeed);
    this->meanCurrent = dir * clampedSpeed;
    this->meanCurrent.Z() = std::clamp(requested.Z(), -0.03, 0.03);
  }

  private: void UpdateCurrent(double _dt,
                              const std::chrono::steady_clock::duration &_simTime)
  {
    const double meanSpeed = std::clamp(this->meanCurrent.Length(), this->minSpeed, this->maxSpeed);
    const double meanHeading = std::atan2(this->meanCurrent.Y(), this->meanCurrent.X());

    const double dSpeed = (meanSpeed - this->currentSpeed) * (_dt / this->speedTau)
      + this->speedNoise * std::sqrt(_dt) * this->normalDist(this->rng);
    this->currentSpeed = std::clamp(this->currentSpeed + dSpeed, this->minSpeed, this->maxSpeed);

    const double headingErr = math::Angle(meanHeading - this->currentHeading).Radian();
    const double dHeading = headingErr * (_dt / this->headingTau)
      + this->headingNoise * std::sqrt(_dt) * this->normalDist(this->rng);
    this->currentHeading += dHeading;

    const double vertical = std::clamp(this->meanCurrent.Z() +
      this->verticalNoise * std::sqrt(_dt) * this->normalDist(this->rng), -0.03, 0.03);

    this->currentVelocityWorld = math::Vector3d(
      std::cos(this->currentHeading) * this->currentSpeed,
      std::sin(this->currentHeading) * this->currentSpeed,
      vertical);

    this->simTimeSec = std::chrono::duration<double>(_simTime).count();
  }

  private: void ApplyHydrodynamicForces(EntityComponentManager &_ecm)
  {
    for (const auto &[linkEntity, st] : this->links)
    {
      if (st.staticModel)
        continue;

      Link link(linkEntity);
      link.EnableVelocityChecks(_ecm, true);
      const auto linOpt = link.WorldLinearVelocity(_ecm);
      if (!linOpt.has_value())
        continue;

      const math::Vector3d relVel = this->currentVelocityWorld - *linOpt;
      math::Vector3d accelCmd = relVel * this->linearDragCoef;
      if (linkEntity == this->cableLinkEntity)
        accelCmd *= this->cableLinearDragScale;
      if (accelCmd.Length() > this->maxAccel)
        accelCmd = accelCmd.Normalized() * this->maxAccel;

      const math::Vector3d force = accelCmd * st.mass;
      link.AddWorldForce(_ecm, force);
    }
  }

  private: void ApplyCableSway(EntityComponentManager &_ecm,
                               double,
                               const std::chrono::steady_clock::duration &)
  {
    if (this->cableLinkEntity == kNullEntity || this->cableSegmentLocalPoints.empty())
      return;

    Link cableLink(this->cableLinkEntity);
    cableLink.EnableVelocityChecks(_ecm, true);
    const auto cableVel = cableLink.WorldLinearVelocity(_ecm).value_or(math::Vector3d::Zero);

    math::Vector3d flowDir = this->currentVelocityWorld;
    flowDir.Z() = 0.0;
    if (flowDir.Length() < kEps)
      return;
    flowDir.Normalize();

    const math::Vector3d up(0, 0, 1);
    math::Vector3d lateral = up.Cross(flowDir);
    if (lateral.Length() < kEps)
      return;
    lateral.Normalize();

    const double flowSpeed = std::clamp(this->currentVelocityWorld.Length(), this->minSpeed, this->maxSpeed);
    const double segmentForceBase = std::clamp(
      this->cableForceScale * flowSpeed * flowSpeed, 0.0, this->cableMaxForcePerSegment);

    for (const auto &seg : this->cableSegmentLocalPoints)
    {
      const double phase = 0.35 * static_cast<double>(seg.index);
      const double wave = std::sin((kTwoPi * this->cableSwayFrequency * this->simTimeSec) + phase);
      const double swayTerm = (this->cableSwayAmplitude > 0.0)
        ? (wave * (seg.index % 2 == 0 ? 1.0 : -1.0))
        : 0.0;

      const double dampingAlongLateral = cableVel.Dot(lateral);
      const double perSegmentForceMag = std::clamp(
        segmentForceBase * (1.0 + 0.35 * swayTerm) - this->cableSwayDamping * dampingAlongLateral,
        -this->cableMaxForcePerSegment,
        this->cableMaxForcePerSegment);

      const math::Vector3d forceWorld = lateral * perSegmentForceMag;
      cableLink.AddWorldForce(_ecm, forceWorld, seg.localPos);
    }
  }

  private: void ApplyCableAnchor(EntityComponentManager &_ecm)
  {
    if (this->cableLinkEntity == kNullEntity || !this->hasCableReferencePose)
      return;

    const bool useLinearAnchor =
      this->cableAnchorLinearStiffness > 0.0 || this->cableAnchorLinearDamping > 0.0;
    const bool useAngularAnchor =
      this->cableAnchorAngularStiffness > 0.0 || this->cableAnchorAngularDamping > 0.0;
    if (!useLinearAnchor && !useAngularAnchor)
      return;

    Link cableLink(this->cableLinkEntity);
    cableLink.EnableVelocityChecks(_ecm, true);
    const auto poseOpt = cableLink.WorldPose(_ecm);
    const auto linVelOpt = cableLink.WorldLinearVelocity(_ecm);
    const auto angVelOpt = cableLink.WorldAngularVelocity(_ecm);
    if (!poseOpt.has_value() || !linVelOpt.has_value() || !angVelOpt.has_value())
      return;

    if (useLinearAnchor)
    {
      math::Vector3d force =
        (this->cableReferencePose.Pos() - poseOpt->Pos()) * this->cableAnchorLinearStiffness
        - (*linVelOpt * this->cableAnchorLinearDamping);
      if (force.Length() > this->cableAnchorMaxForce)
        force = force.Normalized() * this->cableAnchorMaxForce;
      cableLink.AddWorldForce(_ecm, force);
    }

    if (useAngularAnchor)
    {
      const math::Vector3d refRpy = this->cableReferencePose.Rot().Euler();
      const math::Vector3d curRpy = poseOpt->Rot().Euler();
      math::Vector3d angErr(
        math::Angle(refRpy.X() - curRpy.X()).Radian(),
        math::Angle(refRpy.Y() - curRpy.Y()).Radian(),
        math::Angle(refRpy.Z() - curRpy.Z()).Radian());

      math::Vector3d torque =
        angErr * this->cableAnchorAngularStiffness
        - (*angVelOpt * this->cableAnchorAngularDamping);
      if (torque.Length() > this->cableAnchorMaxTorque)
        torque = torque.Normalized() * this->cableAnchorMaxTorque;
      cableLink.AddWorldWrench(_ecm, math::Vector3d::Zero, torque);
    }
  }

  private: void PublishCurrent()
  {
    msgs::Vector3d msg;
    msg.set_x(this->currentVelocityWorld.X());
    msg.set_y(this->currentVelocityWorld.Y());
    msg.set_z(this->currentVelocityWorld.Z());
    this->pub.Publish(msg);
  }

  private: void DiscoverLinks(EntityComponentManager &_ecm, bool)
  {
    this->links.clear();

    _ecm.Each<components::Link, components::ParentEntity, components::Name>(
      [&](const Entity &_link,
          const components::Link *,
          const components::ParentEntity *_parent,
          const components::Name *_name)->bool
      {
        const Entity parentModel = _parent->Data();
        const auto *modelComp = _ecm.Component<components::Model>(parentModel);
        if (!modelComp)
          return true;

        const auto *modelNameComp = _ecm.Component<components::Name>(parentModel);
        if (!modelNameComp)
          return true;

        if (!std::regex_match(modelNameComp->Data(), this->modelRegex))
          return true;

        bool isStaticModel = false;
        if (const auto *staticComp = _ecm.Component<components::Static>(parentModel))
          isStaticModel = staticComp->Data();

        double mass = 30.0;
        if (const auto *inertial = _ecm.Component<components::Inertial>(_link))
          mass = std::max(1.0, inertial->Data().MassMatrix().Mass());

        this->links[_link] = LinkState{parentModel, _name->Data(), mass, isStaticModel};
        return true;
      });
  }

  private: void DiscoverCableSegments(EntityComponentManager &_ecm, bool)
  {
    this->cableLinkEntity = kNullEntity;
    this->cableSegmentLocalPoints.clear();

    Entity cableModel = kNullEntity;
    _ecm.Each<components::Model, components::Name>(
      [&](const Entity &_entity,
          const components::Model *,
          const components::Name *_name)->bool
      {
        if (_name->Data() == this->cableModelName)
        {
          cableModel = _entity;
          return false;
        }
        return true;
      });

    if (cableModel == kNullEntity)
      return;

    Model model(cableModel);
    this->cableLinkEntity = model.LinkByName(_ecm, "cable_link");
    if (this->cableLinkEntity == kNullEntity)
      return;

    std::vector<std::pair<std::size_t, math::Vector3d>> indexedPoses;

    _ecm.Each<components::Collision, components::ParentEntity, components::Name, components::Pose>(
      [&](const Entity &,
          const components::Collision *,
          const components::ParentEntity *_parent,
          const components::Name *_name,
          const components::Pose *_pose)->bool
      {
        if (_parent->Data() != this->cableLinkEntity)
          return true;
        if (!std::regex_match(_name->Data(), this->segmentRegex))
          return true;

        std::size_t idx = this->ExtractTrailingIndex(_name->Data());
        indexedPoses.emplace_back(idx, _pose->Data().Pos());
        return true;
      });

    std::sort(indexedPoses.begin(), indexedPoses.end(),
      [](const auto &_a, const auto &_b) { return _a.first < _b.first; });

    for (const auto &entry : indexedPoses)
      this->cableSegmentLocalPoints.push_back(SegmentPoint{entry.second, entry.first});
  }

  private: void CacheCableReferencePose(EntityComponentManager &_ecm)
  {
    if (this->cableLinkEntity == kNullEntity || this->hasCableReferencePose)
      return;

    Link cableLink(this->cableLinkEntity);
    const auto poseOpt = cableLink.WorldPose(_ecm);
    if (!poseOpt.has_value())
      return;

    this->cableReferencePose = *poseOpt;
    this->hasCableReferencePose = true;
  }

  private: std::size_t ExtractTrailingIndex(const std::string &_s) const
  {
    std::size_t end = _s.size();
    while (end > 0 && std::isdigit(static_cast<unsigned char>(_s[end - 1])))
      --end;
    if (end == _s.size())
      return std::numeric_limits<std::size_t>::max();
    return static_cast<std::size_t>(std::stoul(_s.substr(end)));
  }

  private: std::string GetString(const std::shared_ptr<const sdf::Element> &_sdf,
                                 const std::string &_key,
                                 const std::string &_def) const
  {
    if (_sdf && _sdf->HasElement(_key))
      return _sdf->Get<std::string>(_key);
    return _def;
  }

  private: double GetDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                            const std::string &_key,
                            double _def) const
  {
    if (_sdf && _sdf->HasElement(_key))
      return _sdf->Get<double>(_key);
    return _def;
  }

  private: int GetInt(const std::shared_ptr<const sdf::Element> &_sdf,
                      const std::string &_key,
                      int _def) const
  {
    if (_sdf && _sdf->HasElement(_key))
      return _sdf->Get<int>(_key);
    return _def;
  }

  private: transport::Node node;
  private: transport::Node::Publisher pub;

  private: std::map<Entity, LinkState> links;
  private: Entity cableLinkEntity{kNullEntity};
  private: std::vector<SegmentPoint> cableSegmentLocalPoints;

  private: std::regex modelRegex{std::regex(".*")};
  private: std::regex segmentRegex{std::regex("segment_collision_.*")};
  private: std::string cableModelName{"submarine_cable"};

  private: math::Vector3d meanCurrent{0.24, 0, 0};
  private: math::Vector3d currentVelocityWorld{0.24, 0, 0};
  private: double currentSpeed{0.24};
  private: double currentHeading{0.0};
  private: double simTimeSec{0.0};

  private: double minSpeed{0.1};
  private: double maxSpeed{0.5};
  private: double speedTau{55.0};
  private: double headingTau{70.0};
  private: double speedNoise{0.02};
  private: double headingNoise{0.03};
  private: double verticalNoise{0.004};

  private: double linearDragCoef{0.42};
  private: double maxAccel{0.35};
  private: double cableLinearDragScale{1.0};

  private: double cableForceScale{0.22};
  private: double cableSwayAmplitude{0.03};
  private: double cableSwayFrequency{0.09};
  private: double cableSwayDamping{1.6};
  private: double cableMaxForcePerSegment{2.2};
  private: math::Pose3d cableReferencePose;
  private: bool hasCableReferencePose{false};
  private: double cableAnchorLinearStiffness{0.0};
  private: double cableAnchorLinearDamping{0.0};
  private: double cableAnchorAngularStiffness{0.0};
  private: double cableAnchorAngularDamping{0.0};
  private: double cableAnchorMaxForce{1e9};
  private: double cableAnchorMaxTorque{1e9};

  private: std::chrono::steady_clock::duration lastDiscoverSimTime{std::chrono::steady_clock::duration::zero()};
  private: std::chrono::steady_clock::duration lastPublishSimTime{std::chrono::steady_clock::duration::zero()};
  private: std::chrono::milliseconds discoverPeriod{1500};
  private: std::chrono::milliseconds publishPeriod{300};

  private: std::mt19937 rng;
  private: std::normal_distribution<double> normalDist{0.0, 1.0};
};

}  // namespace ocean_current_plugin

GZ_ADD_PLUGIN(ocean_current_plugin::CurrentSystem,
              gz::sim::System,
              ocean_current_plugin::CurrentSystem::ISystemConfigure,
              ocean_current_plugin::CurrentSystem::ISystemPreUpdate)
