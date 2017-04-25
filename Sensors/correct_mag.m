function Sensor = correct_mag(Sensor, BATT_Curr, corMag, mot_comp)

if corMag == 1
    Sensor.mag = Sensor.mag  - mot_comp*(BATT_Curr-2.7635); % subtract the approx initial battery current.
end