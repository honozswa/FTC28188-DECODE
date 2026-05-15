package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Otos;

//@TeleOp(name = "Tuner-OtosAngular", group = "Tuner")
public class otosAngularTuner extends OpMode {

    Otos otos = new Otos();

    @Override
    public void init() {
        otos.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
            otos.resetPos();
        }

        telemetry.addData("Heading",otos.getOtosHeading());
    }
}
