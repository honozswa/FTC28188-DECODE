package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tuner-FlywheelPIDF", group = "Tuner")
public class pidfTuner extends OpMode {

    DcMotorEx shootMotor, shootMotor2;
    DcMotor intakeMotor, gateMotor;
    Servo GateServo;
    double lowVelocity = 1200;
    double midVelocity = 1700;
    double highVelocity = 1700;
    double curTargetVel = midVelocity;
    double F = 0;
    double P = 0;
    double D = 0;
    double[] stepSizes = {10,1,0.1,0.01,0.001,0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
//        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//        gateMotor = hardwareMap.get(DcMotor.class, "gateMotor");
//        GateServo = hardwareMap.get(Servo.class, "GateServo");

        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);
//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
//        gateMotor.setDirection(DcMotor.Direction.FORWARD);

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(P, 0, D, F);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        telemetry.addLine("init Complete");
    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (curTargetVel == midVelocity) {
                curTargetVel = lowVelocity;
            } else { curTargetVel = midVelocity; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad2.dpadUpWasPressed()) {
            D += stepSizes[stepIndex];
        }

        if (gamepad2.dpadDownWasPressed()) {
            D -= stepSizes[stepIndex];
        }

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(P, 0, D, F);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        shootMotor.setVelocity(curTargetVel);
        shootMotor2.setVelocity(curTargetVel);

        double curVel = shootMotor.getVelocity();
        double error = curTargetVel - curVel;

//        if (gamepad1.right_bumper) {
//            GateServo.setPosition(0);
//            intakeMotor.setPower(1);
//            gateMotor.setPower(1);
//        } else {
//            intakeMotor.setPower(0);
//            gateMotor.setPower(0);
//        }

        telemetry.addData("Target Vel", curTargetVel);
        telemetry.addData("Current Vel","%.2f", curVel);
        telemetry.addData("Error","%.2f", error);
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)", F);
        telemetry.addData("Tuning D","%.4f (D-Pad2 U/D)", D);
        telemetry.addData("Step Size","%.4f (B-Button)", stepSizes[stepIndex]);
    }
}
