// BumpableHuman — gz-sim system plugin that makes scripted human actors
// physically solid for the rovers.
//
// SDF <actor> elements have NO collision (robots drive straight through them
// and no contact is registered).  This plugin fixes that the cheap way: for
// every (actor, bumper) pair in the plugin config it keeps an INVISIBLE
// STATIC cylinder model glued to the actor's world pose every simulation
// step.  The cylinder has a collision volume, so the rovers physically bump
// against it (and the contact sensor / bump_counter sees the contact), while
// the human visual stays the same scripted actor.
//
// World usage (see human_world.sdf):
//   <plugin filename="libleo_gz_bumpable_human_plugin.so"
//           name="leo_gz::BumpableHuman">
//     <actor name="human_1" model="human_1_bumper"/>
//     <actor name="human_3" model="human_3_bumper"/>
//     <actor name="human_4" model="human_4_bumper"/>
//   </plugin>
//
// The bumper models should be <static>true</static> with an invisible
// cylinder collision (radius ~0.4 m, length ~1.5 m) so it spans from the
// robot's contact height up to the human body.

#ifdef USE_IGNITION
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/plugin/Register.hh>

namespace gazebo = ignition::gazebo;

#define GZ_ADD_PLUGIN IGNITION_ADD_PLUGIN
#else
#define SUPPRESS_IGNITION_HEADER_DEPRECATION
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components.hh>
#include <gz/plugin/Register.hh>

namespace gazebo = gz::sim;

#endif

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>

namespace leo_gz
{

/// One SDF waypoint of an actor's scripted trajectory.
struct Waypoint
{
  double time{0.0};
  ignition::math::Pose3d pose;
};

/// Scripted trajectory of a human actor, parsed from the actor's SDF DOM.
struct ActorTrajectory
{
  double delay{0.0};    // <delay_start>
  bool loop{true};      // <loop>
  double period{0.0};   // last waypoint time, used to wrap when looping
  std::vector<Waypoint> waypoints;
};

class BumpableHuman
  : public gazebo::System,
    public gazebo::ISystemConfigure,
    public gazebo::ISystemUpdate
{
  /// A pending <actor> config entry (entities not created yet at Configure).
  struct Pending
  {
    std::string actorName;
    std::string modelName;
    ActorTrajectory trajectory;
  };
  std::vector<Pending> pending_;

  /// Resolved actor -> (bumper model entity, trajectory).
  struct Glued
  {
    gazebo::Entity bumper;
    ActorTrajectory trajectory;
  };
  std::unordered_map<gazebo::Entity, Glued> glued_;

  /// Linearly interpolate the scripted pose at time _t (trajectory time).
  static ignition::math::Pose3d Interpolate(
      const ActorTrajectory & _traj, double _t);

public:
  void Configure(const gazebo::Entity & /*_entity*/,
                 const std::shared_ptr<const sdf::Element> & _sdf,
                 gazebo::EntityComponentManager & /*_ecm*/,
                 gazebo::EventManager & /*_eventMgr*/) override
  {
    // Each <actor name="human_X" model="human_X_bumper"/> pair glues one
    // invisible static cylinder to one scripted human actor.
    for (auto actorElem = _sdf->FindElement("actor");
         actorElem;
         actorElem = actorElem->GetNextElement("actor")) {
      auto nameAttr = actorElem->GetAttribute("name");
      auto modelAttr = actorElem->GetAttribute("model");
      if (!nameAttr || !modelAttr) {
        ignerr << "BumpableHuman: <actor> needs name + model attributes"
               << std::endl;
        continue;
      }
      Pending p;
      p.actorName = nameAttr->GetAsString();
      p.modelName = modelAttr->GetAsString();
      pending_.push_back(std::move(p));
    }
    ignmsg << "BumpableHuman: " << pending_.size()
           << " actor/bumper pair(s) queued" << std::endl;
  }

  void Update(const gazebo::UpdateInfo & _info,
              gazebo::EntityComponentManager & _ecm) override
  {
    // 1) Lazy-resolve any actor/bumper pairs whose entities were not created
    //    yet when this world plugin configured.
    if (!pending_.empty()) {
      auto it = pending_.begin();
      while (it != pending_.end()) {
        const std::string & actorName = it->actorName;
        const std::string & modelName = it->modelName;

        auto actorEntity =
            _ecm.EntityByComponents(gazebo::components::Name(actorName));
        auto bumperEntity =
            _ecm.EntityByComponents(gazebo::components::Name(modelName));
        if (actorEntity == gazebo::kNullEntity ||
            bumperEntity == gazebo::kNullEntity) {
          ++it;  // not created yet; retry next step
          continue;
        }

        // Guarantee the bumper is static/immovable (physics will not fight
        // our teleport each step) even if the SDF tag is missing.
        if (!_ecm.Component<gazebo::components::Static>(bumperEntity)) {
          _ecm.CreateComponent(bumperEntity,
                               gazebo::components::Static(true));
        }

        // Parse the actor's scripted trajectory from its SDF DOM so we can
        // compute its walked pose ourselves (Fortress keeps no pose component
        // updated on the actor entity).
        ActorTrajectory traj = it->trajectory;
        auto actorComp =
            _ecm.Component<gazebo::components::Actor>(actorEntity);
        if (actorComp) {
          const sdf::Actor & actorSdf = actorComp->Data();
          traj.loop = actorSdf.ScriptLoop();
          traj.delay = actorSdf.ScriptDelayStart();
          if (actorSdf.TrajectoryCount() > 0) {
            const sdf::Trajectory * trajSdf =
                actorSdf.TrajectoryByIndex(0);
            if (trajSdf) {
              for (uint64_t w = 0; w < trajSdf->WaypointCount(); ++w) {
                const sdf::Waypoint * wp = trajSdf->WaypointByIndex(w);
                if (wp) {
                  traj.waypoints.push_back({wp->Time(), wp->Pose()});
                }
              }
              if (!traj.waypoints.empty()) {
                traj.period = traj.waypoints.back().time;
              }
            }
          }
        }

        const std::size_t nWp = traj.waypoints.size();
        glued_[actorEntity] = Glued{bumperEntity, std::move(traj)};
        ignmsg << "BumpableHuman: gluing bumper '" << modelName
               << "' to actor '" << actorName
               << "' (" << nWp << " waypoints)" << std::endl;
        it = pending_.erase(it);
      }
    }

    // 2) Each step: compute the actor's walked pose from its scripted
    //    trajectory and command the bumper to it.
    //
    //    IMPORTANT: we must use WorldPoseCmd, NOT raw Pose. gz-sim's physics
    //    only repositions models (including static ones) through this command
    //    component ("Process pose commands for static models here, as one-time
    //    changes"). Writing Pose directly only moves the scene/ECM pose and
    //    leaves the collision volume behind, so the rover would drive right
    //    through the human. The physics system removes the command after one
    //    iteration, so we (re)create/refresh it every step.
    for (const auto & pair : glued_) {
      const gazebo::Entity actor = pair.first;
      const Glued & glued = pair.second;
      const gazebo::Entity bumper = glued.bumper;
      const ActorTrajectory & traj = glued.trajectory;
      if (traj.waypoints.empty()) {
        continue;
      }

      // gz-sim's actor system tracks the current animation time; use it if
      // present, otherwise fall back to sim time minus the start delay.
      double t;
      auto animComp =
          _ecm.Component<gazebo::components::AnimationTime>(actor);
      if (animComp) {
        t = std::chrono::duration<double>(animComp->Data()).count();
      } else {
        t = std::chrono::duration<double>(_info.simTime).count() -
            traj.delay;
      }
      // Wrap to the loop period (harmless if gz-sim already wrapped it).
      if (traj.loop && traj.period > 0.0) {
        t = std::fmod(t, traj.period);
      }
      if (t < 0.0) {
        t = 0.0;
      }

      const ignition::math::Pose3d pose = Interpolate(traj, t);
      if (_ecm.Component<gazebo::components::WorldPoseCmd>(bumper)) {
        _ecm.SetComponentData<gazebo::components::WorldPoseCmd>(bumper, pose);
      } else {
        _ecm.CreateComponent(bumper, gazebo::components::WorldPoseCmd(pose));
      }
    }
  }
};

//////////////////////////////////////////////////
ignition::math::Pose3d BumpableHuman::Interpolate(
    const ActorTrajectory & _traj, double _t)
{
  const auto & wps = _traj.waypoints;
  if (wps.empty()) {
    return ignition::math::Pose3d::Zero;
  }
  if (_t <= wps.front().time) {
    return wps.front().pose;
  }
  if (_t >= wps.back().time) {
    return wps.back().pose;
  }
  for (std::size_t i = 0; i + 1 < wps.size(); ++i) {
    if (_t >= wps[i].time && _t <= wps[i + 1].time) {
      const double span = wps[i + 1].time - wps[i].time;
      const double alpha = span > 0.0 ? (_t - wps[i].time) / span : 0.0;
      const auto & p0 = wps[i].pose;
      const auto & p1 = wps[i + 1].pose;
      ignition::math::Pose3d out;
      out.Pos() = p0.Pos() * (1.0 - alpha) + p1.Pos() * alpha;
      // Waypoints here all have identity yaw; keep the first one's rotation.
      out.Rot() = p0.Rot();
      return out;
    }
  }
  return wps.back().pose;
}

}  // namespace leo_gz

GZ_ADD_PLUGIN(
  leo_gz::BumpableHuman,
  gazebo::System,
  leo_gz::BumpableHuman::ISystemConfigure,
  leo_gz::BumpableHuman::ISystemUpdate)
