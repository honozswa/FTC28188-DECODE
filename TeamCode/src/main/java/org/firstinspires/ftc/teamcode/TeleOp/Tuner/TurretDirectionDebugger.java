package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Turret;

@TeleOp(name = "Tuner-TurretDirection", group = "Tuner")
public class TurretDirectionDebugger extends OpMode {
    Turret turret = new Turret();

    @Override
    public void init() {
        turret.init(hardwareMap);
    }

    @Override
    public void loop() {
        double power = -gamepad1.left_stick_y;

        turret.setPower(power);

        telemetry.addLine("If power is positive, turret should turn left");
        telemetry.addData("Angle", turret.getCurrentAngle());
        telemetry.addData("Power", power);
    }

}
