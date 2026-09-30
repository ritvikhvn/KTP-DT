-- imu_param_sweep.lua
-- Time-based sequential SITL IMU fault sweep.
-- Phases: ACC1_BIAS -> ACC1_RND(alone) -> VIB_FREQ(alone) -> ACC1_RND+VIB_FREQ
-- Each phase: apply high value, hold, revert to default (0), rest, advance.
-- SITL only (SIM_* params). Starts when waypoint #2 is reached (all timing relative to it);
-- by then motors_on is true, else ACC1_RND / VIB_FREQ have no effect (AP_InertialSensor_SITL.cpp).

-- ===== Config =====
local HOLD_S      = 10     -- seconds to hold each fault
local REST_S      = 5      -- seconds at baseline between phases
local ACC_RND_HI  = 3.0    -- m/s^2  accel motor-noise amplitude
local BIAS_HI     = 1.0    -- m/s^2  accel static bias, per axis
local VIB_FREQ_HI = 40.0   -- Hz     vibration frequency, per axis
local START_WP    = 2      -- begin sweep when this nav waypoint is reached; all timing is relative to it

-- Phase table: name + list of {param_name, high_value}. All defaults are 0.
local PHASES = {
  { name = "ACC1_BIAS", sets = {
      {"SIM_ACC1_BIAS_X", BIAS_HI}, {"SIM_ACC1_BIAS_Y", BIAS_HI}, {"SIM_ACC1_BIAS_Z", BIAS_HI} } },
  { name = "ACC1_RND (alone)", sets = {
      {"SIM_ACC1_RND", ACC_RND_HI} } },
  { name = "VIB_FREQ (alone)", sets = {
      {"SIM_VIB_FREQ_X", VIB_FREQ_HI}, {"SIM_VIB_FREQ_Y", VIB_FREQ_HI}, {"SIM_VIB_FREQ_Z", VIB_FREQ_HI} } },
  { name = "ACC1_RND + VIB_FREQ", sets = {
      {"SIM_ACC1_RND", ACC_RND_HI},
      {"SIM_VIB_FREQ_X", VIB_FREQ_HI}, {"SIM_VIB_FREQ_Y", VIB_FREQ_HI}, {"SIM_VIB_FREQ_Z", VIB_FREQ_HI} } },
}

-- ===== State =====
local idx     = 0       -- current phase index
local applied = false   -- fault currently applied?
local started = false
local t_next  = 0       -- millis() of next transition

local function set_all(sets, value)  -- value=nil uses each entry's high value
  for _, pv in ipairs(sets) do
    local v = (value == nil) and pv[2] or value
    if not param:set(pv[1], v) then
      gcs:send_text(4, string.format("SWEEP: FAILED set %s", pv[1]))
    end
  end
end

gcs:send_text(6, string.format("IMU sweep: loaded, waiting for armed + WP%d", START_WP))

function update()
  if not arming:is_armed() then return update, 1000 end

  if not started then
    if mission:get_current_nav_index() < START_WP then return update, 1000 end
    started = true
    t_next = millis():toint() + REST_S * 1000   -- t0 = WP#2 reached; all timing relative to here
    gcs:send_text(6, string.format("IMU sweep: START at WP%d (baseline rest)", START_WP))
  end

  local now = millis():toint()
  if now < t_next then return update, 500 end

  if applied then
    set_all(PHASES[idx].sets, 0)  -- revert to default
    gcs:send_text(6, string.format("IMU sweep: phase %d %s -> reverted", idx, PHASES[idx].name))
    applied = false
    t_next = now + REST_S * 1000
    if idx >= #PHASES then
      gcs:send_text(6, "IMU sweep: COMPLETE")
      return  -- stop scheduling
    end
  else
    idx = idx + 1
    set_all(PHASES[idx].sets, nil)  -- apply high values
    gcs:send_text(6, string.format("IMU sweep: phase %d %s -> APPLIED (hold %ds)", idx, PHASES[idx].name, HOLD_S))
    applied = true
    t_next = now + HOLD_S * 1000
  end

  return update, 500
end

return update, 1000
