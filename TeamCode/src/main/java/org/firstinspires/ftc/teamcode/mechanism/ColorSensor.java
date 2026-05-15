package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor {
    NormalizedColorSensor sensor, sensor2;
    public enum DetectedColor {
        BALL_DETECTED,
        UNKNOWN
    }
    public enum DetectedColor2 {
        BALL_DETECTED,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        sensor = hwMap.get(NormalizedColorSensor.class,"ColorSensor");
        sensor.setGain(15);

        sensor2 = hwMap.get(NormalizedColorSensor.class,"ColorSensor2");
        sensor2.setGain(15);
    }

    public DetectedColor getDetectedColorRGB(Telemetry telemetry) {
        NormalizedRGBA colors = sensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red;
        normGreen = colors.green;
        normBlue = colors.blue;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        if (normRed < 0.08 && normGreen < 0.13 && normBlue < 0.135) {
            return DetectedColor.BALL_DETECTED;
        }

        return DetectedColor.UNKNOWN;
    }

    public DetectedColor2 getDetectedColorRGB2(Telemetry telemetry) {
        NormalizedRGBA colors = sensor2.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red2", normRed);
        telemetry.addData("green2", normGreen);
        telemetry.addData("blue2", normBlue);

        if (normRed < 0.08 && normGreen < 0.13 && normBlue < 0.135) {
            return DetectedColor2.BALL_DETECTED;
        }

        return DetectedColor2.UNKNOWN;
    }

    public DetectedColor getDetectedBall() {
        NormalizedRGBA colors = sensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red;
        normGreen = colors.green;
        normBlue = colors.blue;

        if (normGreen > 0.3 || normBlue > 0.3 || normRed > 0.3) {
            return DetectedColor.BALL_DETECTED;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }

    public DetectedColor2 getDetectedBall2() {
        NormalizedRGBA colors = sensor2.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red;
        normGreen = colors.green;
        normBlue = colors.blue;

        if (normGreen > 0.3 || normBlue > 0.3 || normRed > 0.3) {
            return DetectedColor2.BALL_DETECTED;
        }
        else {
            return DetectedColor2.UNKNOWN;
        }
    }

}
