package org.firstinspires.ftc.teamcode.TeleOp.Tester;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class setPowerTest extends OpMode {

    DcMotorEx motor;
    double power = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class,"shootMotor");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            power += 0.01;
        }
        if (gamepad1.dpad_down) {
            power -= 0.01;
        }

        motor.setPower(power);
    }
}
