-- Test MCP23017 GPIO expander implementation
-- https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf

local expander = i2c:get_device(0, 0x20)
local toggle_state = false

-- B4-B7 as outputs
expander:write_register(0x01, 0x00)

function update()
    if toggle_state then
        expander:write_register(0x13, 0x00)
        toggle_state = false
    else
        expander:write_register(0x13, 0xFF)
        toggle_state = true
    end

    return update, 1000
end


return update, 1000