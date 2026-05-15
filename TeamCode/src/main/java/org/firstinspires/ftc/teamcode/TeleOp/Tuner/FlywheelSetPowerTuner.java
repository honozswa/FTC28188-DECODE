package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.mechanism.FlywheelPowerTest;

//@TeleOp(name = "Tuner-FlywheelSetPowerMethod", group = "Tuner")
public class FlywheelSetPowerTuner extends OpMode {

    FlywheelPowerTest shooter = new FlywheelPowerTest();
    double lowVelocity = 1200;
    double midVelocity = 1600;
    double curTargetVel = midVelocity;
    double I = ShooterConstant.kI;
    double[] stepSizes = {1,0.1,0.01,0.001,0.0001,0.00001,0.000001,0.0000001,0.00000001};
    int stepIndex = 1;

    @Override
    public void init() {
        shooter.init(hardwareMap);
        telemetry.addLine("init Complete");
    }

    @Override
    public void loop() {

        shooter.update();

        if (gamepad1.yWasPressed()) {
            if (curTargetVel == midVelocity) {
                curTargetVel = lowVelocity;
            } else { curTargetVel = midVelocity; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

//        if (gamepad1.dpadLeftWasPressed()) {
//            F -= stepSizes[stepIndex];
//        }
//
//        if (gamepad1.dpadRightWasPressed()) {
//            F += stepSizes[stepIndex];
//        }

        if (gamepad1.dpadUpWasPressed()) {
            I += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            I -= stepSizes[stepIndex];
        }

        I = Math.max(0,I);

        shooter.setkI(I);

        shooter.setTargetVelocity(curTargetVel);

        double curVel = shooter.getVelocity();
        double error = curTargetVel - curVel;

        telemetry.addData("Target Vel", curTargetVel);
        telemetry.addData("Current Vel","%.2f", curVel);
        telemetry.addData("Error","%.2f", error);
        telemetry.addData("Tuning I","%.8f (D-Pad U/D)", shooter.getkI());
//        telemetry.addData("Tuning F","%.8f (D-Pad L/R)", F);
        telemetry.addData("Step Size","%.4f (B-Button)", stepSizes[stepIndex]);
        telemetry.addData("Power", "%.3f",shooter.getPower());
        telemetry.addData("Ready", shooter.atTarget());
    }
}
