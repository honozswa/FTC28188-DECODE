package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Turret;

@TeleOp(name = "Tuner-TurretMinPower", group = "Tuner")
public class MinPowerTuner extends OpMode {

    Turret turret = new Turret();
    double power = 0;
    double[] stepSizes = {0.1,0.01,0.001,0.0001,0.00001,0.000001};
    int stepIndex = 1;

    @Override
    public void init() {
        turret.init(hardwareMap);
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

        turret.setPower(power);

        telemetry.addLine("Increase power (D-Pad U/D) until the turret start to move");
        telemetry.addLine("");
        telemetry.addData("Step Size","%.6f (B-Button)", stepSizes[stepIndex]);
        telemetry.addData("power", power);
        telemetry.addLine("");
        telemetry.addLine("Put this value into MinPowerThreshold in TurretConstant");

        telemetry.addLine("");
        telemetry.addLine("For exponent, find smallest PID output then calculate: exponent = ln(minPower) / ln(SmallestPidOutput)");
    }

}
