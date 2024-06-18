-- Lua script to control Tic stepper driver
-- TODO stepper response CRC
local tic = serial:find_serial(0)
tic:begin(9600)
tic:set_flow_control(0)

TIC_CMD_ENERGIZE = 0x85
TIC_CMD_DEENERGIZE = 0x86
TIC_CMD_SET_VELOCITY = 0xE3
TIC_CMD_GET_VARIABLE = 0xA1

TIC_VAR_UPTIME = 0x35  -- unsigned 32-bit
TIC_VAR_CURR_POS = 0x22  -- signed 32-bit

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


local energized = false
function update()
  -- Flash LED (test quick)
  if energized then
    tic_quick(TIC_CMD_DEENERGIZE)
  else
    tic_quick(TIC_CMD_ENERGIZE)
  end
  energized = not energized

  -- Spin
  -- send_32bit(TIC_CMD_SET_VELOCITY, 2000000)

  -- Get stepper uptime
  local uptime_str = tic_readstring(TIC_CMD_GET_VARIABLE, TIC_VAR_UPTIME, 4)
  if uptime_str ~= nil then
    local uptime = string.unpack("<I4", uptime_str)
    gcs:send_text(6, string.format("TIC uptime: %d", uptime))
  end

  return update, 1000
end


return update, 1000
