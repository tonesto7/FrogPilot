import os
import random

import openpilot.system.sentry as sentry

from cereal import log

from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.events import Events

Desire = log.Desire

class FrogPilotEvents:
  def __init__(self, FrogPilotPlanner):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")

    self.events = Events()
    self.frogpilot_planner = FrogPilotPlanner

    self.fcw_random_event_triggered = False
    self.holiday_theme_alerted = False
    self.openpilot_crashed_triggered = False
    self.previous_traffic_mode = False
    self.random_event_triggered = False
    self.stopped_for_light = False

    self.frame = 0
    self.max_acceleration = 0
    self.previous_v_cruise = 0
    self.random_event_timer = 0

  def update(self, carState, frogpilotCarControl, frogpilotCarState, modelData, v_cruise, frogpilot_toggles):
    if self.random_event_triggered:
      self.random_event_timer += DT_MDL
      if self.random_event_timer >= 4:
        self.random_event_triggered = False
        self.random_event_timer = 0
        self.params_memory.remove("CurrentRandomEvent")

    if self.frogpilot_planner.forcing_stop:
      self.events.add(EventName.forcingStop)

    if frogpilot_toggles.green_light_alert and not self.frogpilot_planner.tracking_lead and carState.standstill:
      if not self.frogpilot_planner.model_stopped and self.stopped_for_light:
        self.events.add(EventName.greenLight)
      self.stopped_for_light = self.frogpilot_planner.cem.stop_light_detected
    else:
      self.stopped_for_light = False

    if not self.holiday_theme_alerted and frogpilot_toggles.current_holiday_theme != 0 and self.frame >= 10:
      self.events.add(EventName.holidayActive)
      self.holiday_theme_alerted = True

    if self.frogpilot_planner.lead_departing:
      self.events.add(EventName.leadDeparting)

    if not self.openpilot_crashed_triggered and os.path.isfile(os.path.join(sentry.CRASHES_DIR, 'error.txt')):
      if frogpilot_toggles.random_events:
        self.events.add(EventName.openpilotCrashedRandomEvent)
      else:
        self.events.add(EventName.openpilotCrashed)
      self.openpilot_crashed_triggered = True

    if frogpilot_toggles.random_events and not self.random_event_triggered:
      acceleration = carState.aEgo

      if not carState.gasPressed:
        self.max_acceleration = max(acceleration, self.max_acceleration)
      else:
        self.max_acceleration = 0

      if 3.5 > self.max_acceleration >= 3.0 and acceleration < 1.5:
        self.events.add(EventName.accel30)
        self.params_memory.put_int("CurrentRandomEvent", 2)
        self.random_event_triggered = True
        self.max_acceleration = 0

      elif 4.0 > self.max_acceleration >= 3.5 and acceleration < 1.5:
        self.events.add(EventName.accel35)
        self.params_memory.put_int("CurrentRandomEvent", 3)
        self.random_event_triggered = True
        self.max_acceleration = 0

      elif self.max_acceleration >= 4.0 and acceleration < 1.5:
        self.events.add(EventName.accel40)
        self.params_memory.put_int("CurrentRandomEvent", 4)
        self.random_event_triggered = True
        self.max_acceleration = 0

      if self.frogpilot_planner.taking_curve_quickly:
        self.events.add(EventName.dejaVuCurve)
        self.params_memory.put_int("CurrentRandomEvent", 5)
        self.random_event_triggered = True

      if frogpilotCarControl.noEntryEventTriggered and not self.no_entry_alert_played:
        self.events.add(EventName.hal9000)
        self.no_entry_alert_played = True
        self.random_event_triggered = True

      if frogpilotCarControl.steerSaturatedEventTriggered:
        event_choices = [1, 2]
        if self.frame % (100 // len(event_choices)) == 0:
          event_choice = random.choice(event_choices)
          if event_choice == 1:
            self.events.add(EventName.firefoxSteerSaturated)
            self.params_memory.put_int("CurrentRandomEvent", 1)
          elif event_choice == 2:
            self.events.add(EventName.goatSteerSaturated)
          self.random_event_triggered = True

      if 70 > v_cruise >= 69 and v_cruise != self.previous_v_cruise:
        self.events.add(EventName.vCruise69)
        self.previous_v_cruise = v_cruise
        self.random_event_triggered = True

      if frogpilotCarControl.fcwEventTriggered and not self.fcw_random_event_triggered:
        self.events.add(EventName.yourFrogTriedToKillMe)
        self.fcw_random_event_triggered = True
        self.random_event_triggered = True

    if frogpilot_toggles.speed_limit_alert and self.speed_limit_changed:
      self.events.add(EventName.speedLimitChanged)

    if self.frame == 5.5 and self.params.get("NNFFModelName", encoding='utf-8') is not None:
      self.events.add(EventName.torqueNNLoad)

    if frogpilotCarState.trafficModeActive != self.previous_traffic_mode:
      if self.previous_traffic_mode:
        self.events.add(EventName.trafficModeInactive)
      else:
        self.events.add(EventName.trafficModeActive)
      self.previous_traffic_mode = frogpilotCarState.trafficModeActive

    if modelData.meta.turnDirection == Desire.turnLeft:
      self.events.add(EventName.turningLeft)
    elif modelData.meta.turnDirection == Desire.turnRight:
      self.events.add(EventName.turningRight)

    self.frame += DT_MDL
