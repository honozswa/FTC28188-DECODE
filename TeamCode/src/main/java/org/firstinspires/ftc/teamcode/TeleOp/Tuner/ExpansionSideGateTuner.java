package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tuner-ExpansionGate", group = "Tuner")
public class ExpansionSideGateTuner extends LinearOpMode {

    Servo testServo;
    double position = 0;

    @Override
    public void runOpMode() {

        testServo = hardwareMap.get(Servo.class, "GateServo"); // change name if needed
        testServo.setPosition(position);

        waitForStart();

        while (opModeIsActive()) {

            position += -gamepad1.left_stick_y * 0.01;
            position = Range.clip(position, 0.0, 1.0);

            testServo.setPosition(position);

            telemetry.addData("Servo Position", position);
            telemetry.update();
        }
    }
}
