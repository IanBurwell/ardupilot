-- Lua script to control uspamr via rc controller
-- TODO stepper response CRC

-- configurable variables
local stepper_rc_channel = 6
local solenoid1_rc_channel = 7
local solenoid2_rc_channel = 8
local solenoid3_rc_channel = 9
local solenoid4_rc_channel = 10

local current_limit_ma = 1000
local max_speed = 80000000 * 2
local home_speed_jog = 40000000
local home_speed_slow = 10000000
local home_slow_dist_v = 0.5
local pot_min = 0.4
local pot_max = 4
local range_in_steps = 0

local joystick_deadzone = 0.15
local leak_sensor_pin = 57
local loop_period_ms = 100

local tic = serial:find_serial(0)
local stepper_pot = analog:channel()
local expander = i2c:get_device(0, 0x20)


-- TIC registers
TIC_CMD_ENERGIZE = 0x85
TIC_CMD_DEENERGIZE = 0x86
TIC_CMD_SET_VELOCITY = 0xE3
TIC_CMD_MAX_SPEED = 0xE6
TIC_CMD_CURRENT_LIMIT = 0x91
TIC_CMD_HALT_SET_POS = 0xEC
TIC_CMD_HALT_HOLD = 0x89
TIC_CMD_GET_VARIABLE = 0xA1
TIC_CMD_SET_TARGET_POS = 0xE0

TIC_VAR_UPTIME = 0x35  -- unsigned 32-bit
TIC_VAR_CURR_POS = 0x22  -- signed 32-bit
TIC_MISC_FLAGS = 0x01  -- unsigned 8-bit

-- https://www.pololu.com/docs/0J71/9
-- Send a tic command (with CRC)
function tic_crc(...)
  local arg = {...}
  local crc = 0
  for i, v in ipairs(arg) do
    crc = crc ~ v;
    for j = 1, 8 do
      if (crc & 1 ~= 0) then
        crc = crc ~ 0x91
      end
      crc = crc >> 1;
    end
  end
  return crc
end
-- Calculates a CRC from a set of bytes
function tic_send(...)
  local arg = {...}
  local crc = tic_crc(...)
  for i, v in ipairs(arg) do
    tic:write(v)
  end
  tic:write(crc)
end
-- Write quick 1-byte command to Tic (ex 0x89 for hault)
function tic_quick(command)
  tic_send(command)
end
-- send a 1-byte command with a 7bit data field
function tic_7bit(command, data)
  tic_send(command, data)
end
-- send a 1-byte command with a 32bit data field (this sucks)
function tic_32bit(command, data)
  -- first byte sent is most significant bits (?!)
  msb1 = (data >> 8*4-1) & 0x01
  msb2 = (data >> 8*3-1) & 0x01
  msb3 = (data >> 8*2-1) & 0x01
  msb4 = (data >> 8*1-1) & 0x01
  MSbs = (msb1 << 3) | (msb2 << 2) | (msb3 << 1) | msb4

  -- next bytes are data bytes without their most significant bits (little-endian)
  data1 = (data >> 8*3) & 0x7F
  data2 = (data >> 8*2) & 0x7F
  data3 = (data >> 8  ) & 0x7F
  data4 = (data       ) & 0x7F

  tic_send(command, MSbs, data4, data3, data2, data1)
end
-- read block data
function tic_readstring(command, offset, length)
  tic_send(command, offset, length)
  -- TODO Sending is asyncronous so we cant expect a responce right away.
  --      This works in a loop for now but is delayed by one cycle

  local responce = tic:readstring(length+1)
  if responce == nil or string.len(responce) ~= length+1 then
    return nil
  end
  local data_bytes = {string.byte(responce,1,-2)}
  local crc = tic_crc(table.unpack(data_bytes))
  if crc ~= string.byte(responce, -1) then
    gcs:send_text(1, "TIC CRC responce incorrect")
    return nil
  end

  return string.sub(responce,1,-1)
end

-- setup tic
tic:begin(9600)
tic:set_flow_control(0)
tic_quick(TIC_CMD_ENERGIZE)
tic_32bit(TIC_CMD_MAX_SPEED, math.floor(max_speed))
tic_32bit(TIC_CMD_CURRENT_LIMIT, math.floor(current_limit_ma))

-- setup ADC
stepper_pot:set_pin(202)
-- set GPIO expander pins low
expander:write_register(0x01, 0x00)


-- Scheduled function: Loop that runs a homing sequence
local min_found = false
function home_tic_loop()
  local pot_volts = stepper_pot:voltage_average()/1000
  gcs:send_text(6, string.format("ADC V: %.2f", pot_volts))

  -- When we hit minimum endstop halt and set home position
  if not min_found and pot_volts <= pot_min then
    gcs:send_text(6, string.format("Found stepper min (%2f)", pot_volts))
    min_found = true
    pot_min = pot_volts
    tic_32bit(TIC_CMD_HALT_SET_POS, 0)
    return home_tic_loop, 500
  end

  -- When we hit maximum endstop halt and get position
  if min_found and pot_volts >= pot_max then
    pot_max = pot_volts
    gcs:send_text(6, string.format("Found stepper max (%2f)", pot_volts))
    tic_quick(TIC_CMD_HALT_HOLD)
    return read_position, 100
  end

  -- Start by finding the minimum endstop
  if not min_found then
    if pot_volts > home_slow_dist_v+pot_min then
      tic_32bit(TIC_CMD_SET_VELOCITY, -math.floor(home_speed_jog))
    else
      tic_32bit(TIC_CMD_SET_VELOCITY, -math.floor(home_speed_slow))
    end

  -- Later find the maximum endstop
  else
    if pot_volts < pot_max-home_slow_dist_v then
      tic_32bit(TIC_CMD_SET_VELOCITY, math.floor(home_speed_jog))
    else
      tic_32bit(TIC_CMD_SET_VELOCITY, math.floor(home_speed_slow))
    end
  end

  return home_tic_loop, loop_period_ms
end


-- Scheduled function: Reads the position of the stepper and updates range_in_steps
function read_position()
  local curr_pos_str = tic_readstring(TIC_CMD_GET_VARIABLE, TIC_VAR_CURR_POS, 4)
  if curr_pos_str == nil then
    return read_position, 100
  end
  range_in_steps = string.unpack("<i4", curr_pos_str)
  gcs:send_text(6, string.format("Stepper homed, range=%d", range_in_steps))

  return control_loop, 100
end


-- Scheduled function: Main update loop
function control_loop()
  -- check leak sensor
  if gpio:read(leak_sensor_pin) then
    gcs:send_text(0, "LEAK DETECTED!!!! ")
  end


  -- control solenoids
  local gpio_bitmask = 0
  if rc:get_pwm(solenoid1_rc_channel) > 1700 then
    gpio_bitmask = gpio_bitmask | (1 << 4)
  end
  if rc:get_pwm(solenoid2_rc_channel) > 1700 then
    gpio_bitmask = gpio_bitmask | (1 << 5)
  end
  if rc:get_pwm(solenoid3_rc_channel) > 1700 then
    gpio_bitmask = gpio_bitmask | (1 << 6)
  end
  if rc:get_pwm(solenoid4_rc_channel) > 1700 then
    gpio_bitmask = gpio_bitmask | (1 << 7)
  end
  expander:write_register(0x13, gpio_bitmask)


  -- check stepper uncertain flag
  local pot_volts = stepper_pot:voltage_average()/1000
  local misc_flags_str = tic_readstring(TIC_CMD_GET_VARIABLE, TIC_MISC_FLAGS, 4)
  if misc_flags_str ~= nil then
    local misc_flags = string.unpack("B", misc_flags_str)
    if misc_flags & (1 << 1) ~= 0 then
      local curr_step_pos = (pot_volts-pot_min)/(pot_max-pot_min) * range_in_steps
      gcs:send_text(1, string.format("TIC uncertain (%02X), setting pos=%d", misc_flags, math.floor(curr_step_pos)))
      tic_32bit(TIC_CMD_HALT_SET_POS, math.floor(curr_step_pos))

      return control_loop, loop_period_ms
    end
  end


  -- control stepper position
  local pwm = rc:get_pwm(stepper_rc_channel)
  local scaled_pwm = (pwm-1500)/400
  gcs:send_text(6, string.format("ADC V: %.2f", pot_volts))

  if pwm ~= 0 then
    tic_32bit(TIC_CMD_SET_TARGET_POS, math.floor((scaled_pwm+1)/2 * range_in_steps))
  else
    tic_32bit(TIC_CMD_SET_VELOCITY, 0)
  end
  -- if pwm ~= 0 and scaled_pwm > joystick_deadzone then
  --   tic_32bit(TIC_CMD_SET_VELOCITY, math.floor(max_speed*scaled_pwm))

  -- elseif pwm ~= 0 and scaled_pwm < -joystick_deadzone then
  --   tic_32bit(TIC_CMD_SET_VELOCITY, math.floor(max_speed*scaled_pwm))

  -- else
  --   tic_32bit(TIC_CMD_SET_VELOCITY, 0)
  -- end

  return control_loop, loop_period_ms
end


return home_tic_loop, 1000
