-- Test ADS1115 ADC implementation


local chan1 = analog:channel()
chan1:set_pin(201)
local chan2 = analog:channel()
chan2:set_pin(202)
local chan3 = analog:channel()
chan3:set_pin(203)
local chan4 = analog:channel()
chan4:set_pin(204)

function update()

    mV_chan1 = chan1:voltage_latest()
    mV_chan2 = chan2:voltage_latest()
    mV_chan3 = chan3:voltage_latest()
    mV_chan4 = chan4:voltage_latest()

    gcs:send_text(0, string.format("ADC: (%4.0f, %4.0f, %4.0f, %4.0f)", mV_chan1, mV_chan2, mV_chan3, mV_chan4))

    return update, 1000
end


return update, 1000