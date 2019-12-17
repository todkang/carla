// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/element/Waypoint.h"

#include <boost/container_hash/hash.hpp>

namespace std {

  using WaypointHash = hash<carla::road::element::Waypoint>;

  WaypointHash::result_type WaypointHash::operator()(const argument_type &waypoint) const {
    WaypointHash::result_type seed = 0u;
    size_t seedt = static_cast<size_t>(seed);
    boost::hash_combine(seedt, waypoint.road_id);
    boost::hash_combine(seedt, waypoint.section_id);
    boost::hash_combine(seedt, waypoint.lane_id);
    boost::hash_combine(seedt, static_cast<float>(std::floor(waypoint.s * 200.0)));
    return seed;
  }

} // namespace std
