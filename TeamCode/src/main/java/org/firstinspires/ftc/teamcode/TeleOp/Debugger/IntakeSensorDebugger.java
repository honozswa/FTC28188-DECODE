package org.firstinspires.ftc.teamcode.TeleOp.Debugger;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.ShooterV6;

@TeleOp(name = "Debugger-IntakeSystem", group = "Debugger")
public class IntakeSensorDebugger extends OpMode {

    ShooterV6 shooter = new ShooterV6();
    boolean intakeOn = false;

    @Override
    public void init() {
        shooter.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Intake
        if (gamepad1.aWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (intakeOn) {
            shooter.intakeOn();
        } else {
            shooter.intakeOff();
        }
        if (gamepad1.x) {
            shooter.setReturnRequested();
        }

        shooter.update();

        telemetry.addData("State", shooter.getState());
        telemetry.addLine("");
        telemetry.addData("lowSensor", shooter.lowSensorDetected());
        telemetry.addData("midSensor", shooter.midSensorDetected());
        telemetry.addData("highSensor", shooter.highSensorDetected());
        telemetry.addLine("");
        telemetry.addData("FullBall", shooter.isThreeBall());
        telemetry.update();

    }
}
