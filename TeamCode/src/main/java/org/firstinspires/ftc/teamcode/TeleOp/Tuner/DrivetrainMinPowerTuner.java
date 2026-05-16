package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;

@TeleOp(name = "Tuner-DrivetrainMinPower", group = "Tuner")
public class DrivetrainMinPowerTuner extends OpMode {

    MecanumDrive drive = new MecanumDrive();
    double power = 0;
    double[] stepSizes = {0.1,0.01,0.001,0.0001,0.00001,0.000001};
    int stepIndex = 1;

    @Override
    public void init() {
        drive.init(hardwareMap);
        telemetry.addLine("init Complete");
    }

    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadUpWasPressed()) {
            power += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            power -= stepSizes[stepIndex];
        }

        drive.drive(0,0,power);

        telemetry.addLine("Increase power (D-Pad U/D) until the robot start to rotate");
        telemetry.addLine("");
        telemetry.addData("Step Size","%.6f (B-Button)", stepSizes[stepIndex]);
        telemetry.addData("power", power);
        telemetry.addLine("");
        telemetry.addLine("Put this value into MinPowerThreshold in ShooterConstant");
        telemetry.addLine("");
        telemetry.addLine("For exponent, find smallest PID output then calculate: exponent = ln(minPower) / ln(SmallestPidOutput)");
        telemetry.addLine("");
        telemetry.addLine("To find PID output, first set the threshold to 0 then run AimbotTuner");
    }

}
