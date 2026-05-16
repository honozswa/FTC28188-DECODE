package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;

@TeleOp(name = "Tuner-ResetPose", group = "Tuner")
public class ResetPose extends OpMode {

    @Override
    public void init() {
        telemetry.addLine("This code is used to reset pose after running autonomous to manually play TeleOp");
        telemetry.addLine("");
        telemetry.addLine("Press start button to reset");
    }

    @Override
    public void loop() {
        PoseConstant.hasAutoPose = false;
        telemetry.addLine("Reset Complete!");
    }

}
