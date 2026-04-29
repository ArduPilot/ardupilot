-- Get UTC date and Time from the HAL
-- You can set BRD_RTC_TYPE for the acceptable time sources (GPS, GCS, Hardware)

-- rtc:get_date_and_time_utc()
-- fills in year, month, day, hour, min, sec and ms
-- year is the regular Gregorian year, month is 0~11, day is 1~31, hour is 0~23, minute is 0~59, second is 0~60 (1 leap second), ms is 0~999
-- values are nil if there is no valid time source 

function update()

    year, month, day, hour, min, sec, ms = rtc:get_date_and_time_utc()
    if year ~= nil then
        gcs:send_text(0,string.format("%i %i %i %i:%i:%i:%i", year, month, day, hour, min, sec, ms))
    end
    

  return update, 1000
end

return update()
