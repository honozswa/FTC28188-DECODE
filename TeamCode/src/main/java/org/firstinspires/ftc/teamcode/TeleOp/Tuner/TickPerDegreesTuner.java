package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Turret;

@TeleOp(name = "Tuner-Tick/Degrees", group = "Tuner")
public class TickPerDegreesTuner extends OpMode {

    Turret turret = new Turret();

    @Override
    public void init() {
        turret.init(hardwareMap);
    }

    @Override
    public void loop() {
        double ticksPerDegrees = turret.getTicks() / 90;

        telemetry.addLine("rotate the turret 90 degrees counter-clockwise");
        telemetry.addLine("---------------------------------------");
        telemetry.addData("Ticks", turret.getTicks());
        telemetry.addData("Ticks/Degrees", ticksPerDegrees);
        telemetry.addLine("Put this number into TICKS_PER_DEGREES in turret class");
        telemetry.addLine("---------------------------------------");
        telemetry.addData("TurretHeading", turret.getCurrentAngle());
    }

}
