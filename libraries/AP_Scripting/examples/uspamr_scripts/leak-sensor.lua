-- Test leak sensor implementation

local sensor_pin = 57

gpio:pinMode(sensor_pin, 0)

function update()
    if not gpio:read(sensor_pin) then
        gcs:send_text(0, "LEAK DETECTED!!!! ")
    end
    return update, 1000
end


return update, 1000