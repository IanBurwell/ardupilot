-- TI BQ40Z BMS shutdown script
---@diagnostic disable: need-check-nil

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local COMMAND_ACK_ID = mavlink_msgs.get_msgid("COMMAND_ACK")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")

local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"

-- Initialize MAVLink rx with number of messages, and buffer depth
mavlink:init(1, 10)

-- Register message id to receive
mavlink:register_rx_msgid(COMMAND_LONG_ID)

local MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
local MAV_RESULT_ACCEPTED = 0

-- Init BMS i2c device
local bms = i2c:get_device(1, 0x0B)

-- Exit emergency shutdown (for BQ40Z60, twice for redundancy)
bms:transfer({0x00, 0xA7, 0x23}, 0)
bms:transfer({0x00, 0xA7, 0x23}, 0)


-- Function that is returned to the AP scheduler when we want to shutdown
function shutdown_loop()
    local ret = bms:transfer({0x00, 0x10, 0x00}, 0)
    if ret == nil then
        gcs:send_text(0, "BQ40Z shutdown transfer failed")
    end

    return shutdown_loop, 500
end

-- Main loop
function update()
    local msg, chan = mavlink:receive_chan()
    local parsed_msg = nil

    if (msg ~= nil) then
        parsed_msg = mavlink_msgs.decode(msg, msg_map)
    end

    if parsed_msg ~= nil and (parsed_msg.msgid == COMMAND_LONG_ID) and (parsed_msg.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) then
        gcs:send_text(0, "Got MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN")

        if parsed_msg.param1 == 2 then
            gcs:send_text(0, "Shutting down BQ40Z!!!")

            -- Send ack
            local ack = {}
            ack.command = parsed_msg.command
            ack.result = MAV_RESULT_ACCEPTED
            ack.progress = 0
            ack.result_param2 = 0
            ack.target_system = parsed_msg.sysid
            ack.target_component = parsed_msg.compid
            mavlink:send_chan(chan, mavlink_msgs.encode("COMMAND_ACK", ack))

            -- Shutdown
            return shutdown_loop, 0
        end
    end

    return update, 1000
end


return update, 1000
