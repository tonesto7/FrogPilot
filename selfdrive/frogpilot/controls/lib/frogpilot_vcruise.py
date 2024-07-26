from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_UNSET

from openpilot.selfdrive.frogpilot.controls.lib.conditional_experimental_mode import PLANNER_TIME
from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_variables import CRUISING_SPEED
from openpilot.selfdrive.frogpilot.controls.lib.map_turn_speed_controller import MapTurnSpeedController
from openpilot.selfdrive.frogpilot.controls.lib.speed_limit_controller import SpeedLimitController

TARGET_LAT_A = 1.9

class FrogPilotVCruise:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.params_memory = Params("/dev/shm/params")

    self.forcing_stop = False
    self.override_force_stop = False
    self.override_slc = False
    self.speed_limit_changed = False

    self.model_length = 0
    self.mtsc_target = 0
    self.overridden_speed = 0
    self.previous_speed_limit = 0
    self.slc_target = 0
    self.tracked_model_length = 0
    self.vtsc_target = 0

  def update(self, carState, controlsState, frogpilotCarControl, frogpilotCarState, frogpilotNavigation, modelData, v_cruise, v_ego, frogpilot_toggles):
    v_cruise_cluster = max(controlsState.vCruiseCluster, v_cruise) * CV.KPH_TO_MS
    v_cruise_diff = v_cruise_cluster - v_cruise

    v_ego_cluster = max(carState.vEgoCluster, v_ego)
    v_ego_diff = v_ego_cluster - v_ego

    # Pfeiferj's Map Turn Speed Controller
    if frogpilot_toggles.map_turn_speed_controller and v_ego > CRUISING_SPEED and controlsState.enabled:
      mtsc_active = self.mtsc_target < v_cruise
      self.mtsc_target = clip(self.mtsc.target_speed(v_ego, carState.aEgo), CRUISING_SPEED, v_cruise)

      if frogpilot_toggles.mtsc_curvature_check and self.road_curvature < 1.0 and not mtsc_active:
        self.mtsc_target = v_cruise
      if self.mtsc_target == CRUISING_SPEED:
        self.mtsc_target = v_cruise
    else:
      self.mtsc_target = v_cruise if v_cruise != V_CRUISE_UNSET else 0

    # Pfeiferj's Speed Limit Controller
    if frogpilot_toggles.speed_limit_controller:
      SpeedLimitController.update(frogpilotCarState.dashboardSpeedLimit, frogpilotNavigation.navigationSpeedLimit, v_cruise, v_ego, frogpilot_toggles)
      unconfirmed_slc_target = SpeedLimitController.desired_speed_limit

      if frogpilot_toggles.speed_limit_confirmation:
        limit_changed = unconfirmed_slc_target != self.previous_speed_limit and abs(self.slc_target - unconfirmed_slc_target) > 1

        speed_limit_decreased = limit_changed and self.previous_speed_limit > unconfirmed_slc_target
        speed_limit_increased = limit_changed and self.previous_speed_limit < unconfirmed_slc_target

        self.previous_speed_limit = unconfirmed_slc_target

        slc_confirmed = False
        if speed_limit_decreased:
          if frogpilot_toggles.speed_limit_confirmation_lower:
            self.speed_limit_changed = True
          else:
            slc_confirmed = True
        elif speed_limit_increased:
          if frogpilot_toggles.speed_limit_confirmation_higher:
            self.speed_limit_changed = True
          else:
            slc_confirmed = True

        if self.speed_limit_changed:
          if frogpilotCarControl.resumePressed or self.params_memory.get_bool("SLCConfirmed"):
            slc_confirmed = True
          elif any(be.type == ButtonType.decelCruise for be in carState.buttonEvents):
            slc_confirmed = False
            self.speed_limit_changed = False

        if self.params_memory.get_bool("SLCConfirmedPressed") or not abs(self.slc_target - unconfirmed_slc_target) > 1:
          self.speed_limit_changed = False
          self.params_memory.put_bool_nonblocking("SLCConfirmed", False)
          self.params_memory.put_bool_nonblocking("SLCConfirmedPressed", False)

        if self.speed_limit_changed:
          self.speed_limit_timer += DT_MDL
          if self.speed_limit_timer >= 10:
            self.speed_limit_changed = False
            self.speed_limit_timer = 0
        else:
          self.speed_limit_timer = 0
      else:
        slc_confirmed = False
        self.speed_limit_changed = False

      if frogpilot_toggles.speed_limit_confirmation and self.speed_limit_changed:
        if slc_confirmed:
          self.slc_target = unconfirmed_slc_target
      else:
        self.slc_target = unconfirmed_slc_target

      self.override_slc = self.overridden_speed > self.slc_target
      self.override_slc |= carState.gasPressed and v_ego > self.slc_target
      self.override_slc &= controlsState.enabled

      if self.override_slc:
        if frogpilot_toggles.speed_limit_controller_override_manual:
          if carState.gasPressed:
            self.overridden_speed = v_ego + v_ego_diff
          self.overridden_speed = clip(self.overridden_speed, self.slc_target, v_cruise + v_cruise_diff)
        elif frogpilot_toggles.speed_limit_controller_override_set_speed:
          self.overridden_speed = v_cruise + v_cruise_diff
      else:
        self.overridden_speed = 0
    else:
      self.slc_target = 0

    # Pfeiferj's Vision Turn Controller
    if frogpilot_toggles.vision_turn_controller and v_ego > CRUISING_SPEED and controlsState.enabled:
      adjusted_road_curvature = self.road_curvature * frogpilot_toggles.curve_sensitivity
      adjusted_target_lat_a = TARGET_LAT_A * frogpilot_toggles.turn_aggressiveness

      self.vtsc_target = (adjusted_target_lat_a / adjusted_road_curvature)**0.5
      self.vtsc_target = clip(self.vtsc_target, CRUISING_SPEED, v_cruise)
    else:
      self.vtsc_target = v_cruise if v_cruise != V_CRUISE_UNSET else 0

    if frogpilot_toggles.force_standstill and carState.standstill and not self.override_force_stop and controlsState.enabled:
      self.forcing_stop = True
      v_cruise = -1

    elif frogpilot_toggles.force_stops and self.cem.stop_light_detected and not self.override_force_stop and controlsState.enabled:
      if self.tracked_model_length == 0:
        self.tracked_model_length = self.model_length

      self.forcing_stop = True
      self.tracked_model_length -= v_ego * DT_MDL
      v_cruise = min((self.tracked_model_length / PLANNER_TIME) - 1, v_cruise)

    else:
      if not self.cem.stop_light_detected:
        self.override_force_stop = False

      self.forcing_stop = False
      self.tracked_model_length = 0

      targets = [self.mtsc_target, max(self.overridden_speed, self.slc_target) - v_ego_diff, self.vtsc_target]
      v_cruise = float(min([target if target > CRUISING_SPEED else v_cruise for target in targets]))

    return v_cruise
