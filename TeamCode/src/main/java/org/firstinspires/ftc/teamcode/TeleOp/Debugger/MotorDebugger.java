package org.firstinspires.ftc.teamcode.TeleOp.Debugger;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Debugger-Motor", group = "Tuner")
public class MotorDebugger extends OpMode {

    DcMotor motor;
    double power = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class,"shootMotor");

        telemetry.addLine("Connect the Motor to the expansion hub port 0");
    }

    @Override
    public void loop() {

        telemetry.addLine("Connect the Motor to the expansion hub port 0");

        if (gamepad1.dpad_up) {
            power += 0.01;
        }
        if (gamepad1.dpad_down) {
            power -= 0.01;
        }

        motor.setPower(power);

        telemetry.addData("power", power);
        telemetry.update();

    }
}
