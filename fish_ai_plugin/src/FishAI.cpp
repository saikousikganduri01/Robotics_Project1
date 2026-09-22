#include <algorithm>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include <chrono>
#include <cctype>
#include <cmath>
#include <mutex>
#include <random>
#include <string>

using namespace gz;
using namespace sim;

class FishAI :
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
  Entity modelEntity{kNullEntity};
  Entity bodyEntity{kNullEntity};
  Entity tailJoint{kNullEntity};

  std::string modelName;

  std::default_random_engine gen;
  std::uniform_real_distribution<double> dist{-1.0, 1.0};

  double timer{0.0};
  double wagTime{0.0};
  math::Vector3d velocity{1, 0, 0};
  double seaweedPhase{0.0};
  double seaweedAmp{0.35};
  transport::Node node;
  mutable std::mutex currentMutex;
  math::Vector3d currentVelocityWorld{0, 0, 0};

public:

  //////////////////////////////////////////////////
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    modelEntity = _entity;

    Model model(_entity);
    modelName = model.Name(_ecm);

    // ---- Detect body links ----
    bodyEntity = model.LinkByName(_ecm, "body");
    if (bodyEntity == kNullEntity)
    {
      for (int i = 1; i <= 200; ++i)
      {
        bodyEntity = model.LinkByName(_ecm, "body" + std::to_string(i));
        if (bodyEntity != kNullEntity)
          break;
      }
    }

    // ---- Detect tail joints (fish only) ----
    tailJoint = model.JointByName(_ecm, "tail_joint");
    if (tailJoint == kNullEntity)
    {
      for (int i = 1; i <= 200; ++i)
      {
        tailJoint = model.JointByName(_ecm, "tail_joint" + std::to_string(i));
        if (tailJoint != kNullEntity)
          break;
      }
    }

    std::random_device rd;
    gen.seed(rd());
    this->node.Subscribe("/ocean_current", &FishAI::OnCurrentMsg, this);

    // Parse numeric suffix for subtle per-model motion variations.
    size_t suffixStart = modelName.size();
    while (suffixStart > 0 &&
           std::isdigit(static_cast<unsigned char>(modelName[suffixStart - 1])))
    {
      --suffixStart;
    }
    if (suffixStart < modelName.size())
    {
      int idx = std::stoi(modelName.substr(suffixStart));
      seaweedPhase = 0.45 * static_cast<double>(idx);
      seaweedAmp = 0.28 + 0.015 * static_cast<double>(idx % 8);
    }
  }

  //////////////////////////////////////////////////
  void OnCurrentMsg(const msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->currentMutex);
    this->currentVelocityWorld.Set(_msg.x(), _msg.y(), _msg.z());
  }

  //////////////////////////////////////////////////
  math::Vector3d CurrentVelocityWorld() const
  {
    std::lock_guard<std::mutex> lock(this->currentMutex);
    return this->currentVelocityWorld;
  }

  //////////////////////////////////////////////////
  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    double dt =
      std::chrono::duration<double>(_info.dt).count();

    wagTime += dt;
    const math::Vector3d current = this->CurrentVelocityWorld();

    // ====================================================
    // 🌿 SEAWEED LOGIC
    // ====================================================
    if (modelName.find("seaweed") != std::string::npos)
    {
      if (bodyEntity == kNullEntity)
        return;

      // Softer and varied two-axis sway for more natural kelp motion.
      const double currentBiasX =
        std::clamp(1.6 * current.Y(), -0.05, 0.05);
      const double currentBiasY =
        std::clamp(-1.6 * current.X(), -0.05, 0.05);
      double waveX = 0.18 * std::sin(1.3 * wagTime + seaweedPhase) + currentBiasX;
      double waveY = seaweedAmp * std::sin(0.9 * wagTime + seaweedPhase) + currentBiasY;

      _ecm.RemoveComponent<components::AngularVelocityCmd>(bodyEntity);
      _ecm.CreateComponent(
          bodyEntity,
          components::AngularVelocityCmd(
              gz::math::Vector3d(waveX, waveY, 0)));

      return;
    }

    // ====================================================
    // 🐟 FISH / SHARK LOGIC
    // ====================================================
    const bool isSwimmer =
      (modelName.find("fish") != std::string::npos) ||
      (modelName.find("shark") != std::string::npos);

    if (isSwimmer)
    {
      if (bodyEntity == kNullEntity)
        return;

      const auto *poseComp = _ecm.Component<components::Pose>(bodyEntity);
      if (!poseComp)
        return;

      math::Vector3d fishPos = poseComp->Data().Pos();

      timer += dt;

      // Change direction every 4 seconds
      if (timer > 4.0)
      {
        velocity.X() = dist(gen);
        velocity.Y() = dist(gen);
        velocity.Z() = dist(gen) * 0.3;

        velocity.Normalize();
        velocity *= 2.0;

        timer = 0.0;
      }

      // Rock-only obstacle avoidance.
      math::Vector3d avoid{0, 0, 0};
      const double detectRadius = 2.2;
      _ecm.Each<components::Model, components::Name, components::Pose>(
          [&](const Entity &,
              const components::Model *,
              const components::Name *_name,
              const components::Pose *_pose) -> bool
          {
            const std::string &candidateName = _name->Data();
            if (candidateName.rfind("rock", 0) != 0)
              return true;

            math::Vector3d away = fishPos - _pose->Data().Pos();
            double dist = away.Length();
            if (dist > 1e-6 && dist < detectRadius)
            {
              away.Normalize();
              double w = (detectRadius - dist) / detectRadius;
              avoid += away * w;
            }
            return true;
          });

      if (avoid.Length() > 1e-6)
      {
        avoid.Normalize();
        math::Vector3d desired = velocity + (2.4 * avoid);
        if (desired.Length() < 1e-6)
          desired = math::Vector3d(1, 0, 0);
        desired.Normalize();
        velocity = desired * 2.0;
      }

      math::Vector3d cmdVelocity = velocity;
      cmdVelocity += math::Vector3d(current.X() * 0.55,
                                    current.Y() * 0.55,
                                    current.Z() * 0.20);

      // Move fish body
      _ecm.RemoveComponent<components::LinearVelocityCmd>(bodyEntity);
      _ecm.CreateComponent(
          bodyEntity,
          components::LinearVelocityCmd(cmdVelocity));

      // Tail wag via JOINT
      if (tailJoint != kNullEntity)
      {
        double wag = 3.0 * sin(10.0 * wagTime);

        _ecm.RemoveComponent<components::JointVelocityCmd>(tailJoint);
        _ecm.CreateComponent(
            tailJoint,
            components::JointVelocityCmd({wag}));
      }

      return;
    }
  }
};

GZ_ADD_PLUGIN(
  FishAI,
  System,
  FishAI::ISystemConfigure,
  FishAI::ISystemPreUpdate
)
