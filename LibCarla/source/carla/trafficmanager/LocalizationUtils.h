// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/client/Actor.h"
#include "carla/client/Vehicle.h"
#include "carla/geom/Location.h"
#include "carla/road/RoadTypes.h"
#include "carla/rpc/ActorId.h"

#include "carla/trafficmanager/SimpleWaypoint.h"

namespace carla {
namespace traffic_manager {

  namespace cc = carla::client;
  namespace cg = carla::geom;
  using Actor = carla::SharedPtr<cc::Actor>;
  using ActorId = carla::ActorId;
  using ActorIdSet = std::unordered_set<ActorId>;
  using SimpleWaypointPtr = std::shared_ptr<SimpleWaypoint>;
  using Buffer = std::deque<SimpleWaypointPtr>;
  using GeoGridId = carla::road::JuncId;

  class TrackTraffic{

  private:
    /// Structure to keep track of overlapping waypoints between vehicles.
    using WaypointOverlap = std::unordered_map<uint64_t, ActorIdSet>;
    WaypointOverlap waypoint_overlap_tracker;
    /// Structure to keep track of vehicles with overlapping paths.
    std::unordered_map<ActorId, ActorIdSet> overlapping_vehicles;
    /// Stored vehicle id set record.
    ActorIdSet actor_id_set_record;
    /// Geodesic grids occupied by actors's paths.
    std::unordered_map<ActorId, std::unordered_set<GeoGridId>> actor_to_grids;
    /// Actors currently passing through grids.
    std::unordered_map<GeoGridId, ActorIdSet> gird_to_actors;

  public:
    TrackTraffic();

    /// Methods to update, remove and retrieve vehicles passing through a waypoint.
    void UpdatePassingVehicle(uint64_t waypoint_id, ActorId actor_id);
    void RemovePassingVehicle(uint64_t waypoint_id, ActorId actor_id);
    ActorIdSet GetPassingVehicles(uint64_t waypoint_id);

    /// Method update grid position of vhehicles based on waypoints being added.
    void UpdateGridPosition(ActorId actor_id, SimpleWaypointPtr waypoint);
    /// Method to remove vehicle from grid associated with the waypoint.
    void RemoveGridPosition(ActorId actor_id, SimpleWaypointPtr removed_waypoint,
                            SimpleWaypointPtr remaining_waypoint);
    /// Method to retreive vehicles with paths sharing common geodesic grids.
    ActorIdSet GetOverlappingVehicles(ActorId actor_id);

  };

  /// Returns the cross product (z component value) between the vehicle's
  /// heading vector and the vector along the direction to the next
  /// target waypoint on the horizon.
  float DeviationCrossProduct(Actor actor, const cg::Location &target_location);
  /// Returns the dot product between the vehicle's heading vector and
  /// the vector along the direction to the next target waypoint on the horizon.
  float DeviationDotProduct(Actor actor, const cg::Location &target_location, bool rear_offset=false);

} // namespace traffic_manager
} // namespace carla