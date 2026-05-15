package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistanceSensor {

    private DigitalChannel sensor;

    public void init(HardwareMap hwMap) {
        sensor = hwMap.get(DigitalChannel.class,"distanceSensor");
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean ballDetection() {
        return sensor.getState();
    }

}
