package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

@TeleOp(name = "Tuner-Hood", group = "Tuner")
public class HoodTuner extends LinearOpMode {

    Servo testServo,testServo2;
    double position = 0;

    @Override
    public void runOpMode() {

        testServo = hardwareMap.get(Servo.class, "HoodServo"); // change name if needed
        testServo2 = hardwareMap.get(Servo.class, "HoodServo2"); // change name if needed

        testServo2.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            position += -gamepad1.left_stick_y * 0.01;
            position = Range.clip(position, 0.0, 1.0);

            testServo.setPosition(position);
            testServo2.setPosition(position + ShooterConstant.hoodServo2Offset);

            telemetry.addData("Servo Position", position);
            telemetry.addData("Servo2 Position", position);
            telemetry.update();
        }
    }
}
