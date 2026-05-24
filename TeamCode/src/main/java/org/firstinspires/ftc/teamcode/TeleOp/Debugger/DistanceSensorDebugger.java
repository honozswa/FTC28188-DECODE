package org.firstinspires.ftc.teamcode.TeleOp.Debugger;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.DistanceSensor;

//@TeleOp(name = "Debugger-DistanceSensor", group = "Tuner")
public class DistanceSensorDebugger extends OpMode {

    DistanceSensor distanceSensor = new DistanceSensor();

    @Override
    public void init() {
            distanceSensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        distanceSensor.ballDetection();
        telemetry.addData("ballDetected", distanceSensor.ballDetection());
        telemetry.update();
    }

}
