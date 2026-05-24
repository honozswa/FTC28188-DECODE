package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Util;

@TeleOp(name = "Tuner-OdometryAim", group = "Tuner")
public class OdometryAimTuner extends OpMode {
    private final MecanumDrive drive = new MecanumDrive();
    Follower follower;

    // ------------------------- PD Controller ------------------------ //
    double kP = ShooterConstant.odokP;
    double kD = ShooterConstant.odokD;
    double Offset = ShooterConstant.odoOffset;
    double lastErrorOdo = 0;
    private static final Pose GOAL = PoseConstant.RED_GOAL;
    private final ElapsedTime timer = new ElapsedTime();

    // ---------------------- driving setup --------------------------------- //
    double forward,strafe,rotate;

    // --------------------- controller based PD tuning ----------------------- //
    double[] stepSizes = {1,0.1,0.01,0.001,0.0001};
    int stepIndex = 2;

    @Override
    public void init() {

        drive.init(hardwareMap);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseConstant.RedCloseAutoStartPose);

    }

    @Override
    public void loop() {

        follower.update();

        // Mecanum Drive Input
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // Auto Align Logic
        Aimbot();

        // Drive
        drive.drive(forward,strafe,rotate);

        // Tuner
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadDownWasPressed()) {
            kP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            kP += stepSizes[stepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            kD -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad2.dpadUpWasPressed()) {
            Offset += Math.toRadians(0.1);
        }
        if (gamepad2.dpadDownWasPressed()) {
            Offset -= Math.toRadians(0.1);
        }

        // Telemetry

        if (gamepad1.left_trigger > 0.5) {
            telemetry.addLine("Auto Align");
        }
        else {
            telemetry.addLine("Manual Rotate Mode");
        }
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)", kP);
        telemetry.addData("Tuning D","%.4f (D-Pad L/R)", kD);
        telemetry.addData("Tuning Offset","%.4f (D-Pad2 U/D)", Math.toDegrees(Offset));
        telemetry.addData("Step Size","%.4f (B Button)", stepSizes[stepIndex]);

    }

    public void Aimbot() {
        if (gamepad1.left_trigger > 0.5) {
            double dt = timer.seconds();
            timer.reset();
            Pose robotPose = follower.getPose();
            double targetHeading = Util.getAngleToGoal(robotPose, GOAL);
            double error = Util.angleWrap(robotPose.getHeading() - targetHeading + Offset);
            double pTerm = error * kP;
            double dTerm = 0;
            if (dt > 0) { dTerm = ((error - lastErrorOdo) / dt) * kD;}
            double rawRotate = Range.clip(pTerm + dTerm, -ShooterConstant.maxRotatePower, ShooterConstant.maxRotatePower);
            if (Math.abs(rawRotate) < ShooterConstant.lowPowerThreshold) {
                rotate = Math.signum(rawRotate) * Math.pow(Math.abs(rawRotate), ShooterConstant.exponent);
            } else {
                rotate = rawRotate;
            }
            lastErrorOdo = error;
            telemetry.addLine("Odometry in use");
            telemetry.addData("error", error);
            telemetry.addData("rotate", rotate);
            telemetry.addLine("");
        }
    }

}
