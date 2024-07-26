from openpilot.common.numpy_fast import clip, interp

from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_variables import CITY_SPEED_LIMIT

TRAFFIC_MODE_BP = [0., CITY_SPEED_LIMIT]

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.slower_lead = False

    self.acceleration_jerk = 0
    self.danger_jerk = 0
    self.speed_jerk = 0
    self.t_follow = 0

  def update(self, controlsState, frogpilotCarState, lead_distance, stopping_distance, v_ego, v_lead, frogpilot_toggles):
    if frogpilotCarState.trafficModeActive:
      self.base_acceleration_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_acceleration)
      self.base_danger_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_danger)
      self.base_speed_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed)
      self.t_follow = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_t_follow)
    else:
      self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
        frogpilot_toggles.aggressive_jerk_acceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed,
        frogpilot_toggles.standard_jerk_acceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed,
        frogpilot_toggles.relaxed_jerk_acceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed,
        frogpilot_toggles.custom_personalities, controlsState.personality
      )
      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities, controlsState.personality
      )

    if self.frogpilot_planner.tracking_lead:
      self.safe_obstacle_distance = int(get_safe_obstacle_distance(v_ego, self.t_follow))
      self.safe_obstacle_distance_stock = self.safe_obstacle_distance
      self.stopped_equivalence_factor = int(get_stopped_equivalence_factor(v_lead))
      self.update_follow_values(lead_distance, stopping_distance, v_ego, v_lead, frogpilot_toggles)
    else:
      self.safe_obstacle_distance = 0
      self.safe_obstacle_distance_stock = 0
      self.stopped_equivalence_factor = 0
      self.acceleration_jerk = self.base_acceleration_jerk
      self.danger_jerk = self.base_danger_jerk
      self.speed_jerk = self.base_speed_jerk

  def update_follow_values(self, lead_distance, stopping_distance, v_ego, v_lead, frogpilot_toggles):
    # Offset by FrogAi for FrogPilot for a more natural approach to a faster lead
    if frogpilot_toggles.aggressive_acceleration and v_lead > v_ego:
      distance_factor = max(lead_distance - (v_ego * self.t_follow), 1)
      standstill_offset = max(stopping_distance - v_ego, 0) * max(v_lead - v_ego, 0)
      acceleration_offset = clip((v_lead - v_ego) + standstill_offset - COMFORT_BRAKE, 1, distance_factor)
      self.acceleration_jerk = self.base_acceleration_jerk / acceleration_offset
      self.danger_jerk = self.base_danger_jerk / acceleration_offset
      self.speed_jerk = self.base_speed_jerk / acceleration_offset
      self.t_follow /= acceleration_offset

    # Offset by FrogAi for FrogPilot for a more natural approach to a slower lead
    if (frogpilot_toggles.conditional_slower_lead or frogpilot_toggles.smoother_braking) and v_lead < v_ego:
      distance_factor = max(lead_distance - (v_lead * self.t_follow), 1)
      far_lead_offset = max(lead_distance - (v_ego * self.t_follow) - stopping_distance + (v_lead - CITY_SPEED_LIMIT), 0)
      braking_offset = clip((v_ego - v_lead) + far_lead_offset - COMFORT_BRAKE, 1, distance_factor)
      if frogpilot_toggles.smoother_braking:
        self.acceleration_jerk = self.base_acceleration_jerk * min(braking_offset, COMFORT_BRAKE / 2)
        self.speed_jerk = self.base_speed_jerk * min(braking_offset, COMFORT_BRAKE * 2)
        self.t_follow /= braking_offset
      self.slower_lead = braking_offset - far_lead_offset > 1
