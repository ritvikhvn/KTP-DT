-- imu_bias_z_0p5.lua
-- Main-study IMU fault injection: single severity rung, SIM_ACC1_BIAS_Z only.
-- Sets the Z-axis accelerometer bias once waypoint #2 is reached; holds for the
-- rest of the flight (no revert -- one severity per run, per the dose-response ladder
-- {0, 0.5, 1.0, 2.0, 5.0} m/s^2). X and Y left at 0 to isolate the Z channel.
-- SITL only. b=0 (nominal baseline) needs no script -- just run without one.

local TARGET_BIAS = -0.5   -- m/s^2, SIM_ACC1_BIAS_Z severity rung
local START_WP     = 168     -- inject once this nav waypoint is reached

local applied = false

gcs:send_text(6, string.format("IMU bias fault: loaded, target %.2f m/s^2 at WP%d", TARGET_BIAS, START_WP))

function update()
  if applied then return update, 5000 end

  if not arming:is_armed() then return update, 1000 end
  if mission:get_current_nav_index() < START_WP then return update, 1000 end

  if not param:set("SIM_ACC1_BIAS_Z", TARGET_BIAS) then
    gcs:send_text(4, "IMU bias fault: FAILED to set SIM_ACC1_BIAS_Z")
    return update, 1000
  end

  gcs:send_text(6, string.format("IMU bias fault: SIM_ACC1_BIAS_Z -> %.2f APPLIED", TARGET_BIAS))
  applied = true
  return update, 5000
end

return update, 1000
