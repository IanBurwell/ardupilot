-- Script that implements a heading-hold manual drive mode for rover
local DEADZONE = 0.05
local TURN_STEER_MULT = 10

-- https://ardupilot.org/rover/docs/parameters.html#mode1-mode1
local MODE_GUIDED = 15
local RC_THR_CHANNEL = 3
local RC_STEER_CHANNEL = 1
local ATC_STR_ANG_P = param:get("ATC_STR_ANG_P")
local target_heading_rads = 0

function update()
    if vehicle:get_mode() ~= MODE_GUIDED or not arming:is_armed() then
        return update, 1000
    end

    local thr = rc:get_channel(RC_THR_CHANNEL):norm_input()
    local steer = rc:get_channel(RC_STEER_CHANNEL):norm_input()

    if steer < -DEADZONE or steer > DEADZONE then
        target_heading_rads = ahrs:get_yaw() + steer * TURN_STEER_MULT
    end

    steer = (target_heading_rads-ahrs:get_yaw())*ATC_STR_ANG_P

    vehicle:set_steering_and_throttle(steer, thr)

    return update, 100
end


return update, 1000
