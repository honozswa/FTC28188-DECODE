package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.ColorSensor;

@TeleOp(name = "Tuner-ColorSensor", group = "Tuner")
public class ColorSensorTuner extends OpMode {
    ColorSensor colorSensor = new ColorSensor();
    @Override
    public void init() {
        colorSensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        colorSensor.getDetectedColorRGB(telemetry);
        colorSensor.getDetectedColorRGB2(telemetry);
    }
}
