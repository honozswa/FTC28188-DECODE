package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tuner-HoodServoOffset", group = "Tuner")
public class HoodServoOffsetTuner extends OpMode {
    private Servo servo1,servo2;
    private double pos1 = 0;
    private double pos2 = 0;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class,"HoodServo");
        servo2 = hardwareMap.get(Servo.class,"HoodServo2");

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        servo1.setPosition(pos1);
        servo2.setPosition(pos2);
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed()) {
            pos1 += 0.01;
        }
        if (gamepad1.dpadDownWasPressed()) {
            pos1 -= 0.01;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            pos2 += 0.01;
        }
        if (gamepad1.dpadRightWasPressed()) {
            pos2 -= 0.01;
        }

        pos1 = Range.clip(pos1,0,1);
        pos2 = Range.clip(pos2,0,1);

        servo1.setPosition(pos1);
        servo2.setPosition(pos2);

        telemetry.addData("Pos1 (Dpad U/D)",pos1);
        telemetry.addData("Pos2 (Dpad L/R)",pos2);
        telemetry.addData("Servo2Offset",pos2-pos1);

    }

}
