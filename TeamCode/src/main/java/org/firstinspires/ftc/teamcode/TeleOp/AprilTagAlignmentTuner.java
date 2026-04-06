package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Webcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilTagAlignmentTuner extends OpMode {
    private final Webcam webcam = new Webcam();
    private final MecanumDrive drive = new MecanumDrive();

    // ------------------------- PD Controller ------------------------ //
    double kP = 0.002;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.4;
    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;

    // ---------------------- driving setup --------------------------------- //
    double forward,strafe,rotate;

    // --------------------- controller based PD tuning ----------------------- //
    double[] stepSizes = {1,0.1,0.01,0.001,0.0001};
    int stepIndex = 2;

    @Override
    public void init() {
        webcam.init(hardwareMap, telemetry);
        drive.init(hardwareMap);

        telemetry.addLine("Initialized");
    }

    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        // Mecanum Drive Input
        forward =- gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // Get April Tag Info
        webcam.update();
        AprilTagDetection id20 = webcam.getTagBySpecificId(20);

        // Auto Align Logic
        if (gamepad1.left_trigger > 0.5) {
            if (id20 != null) {
                error = goalX - id20.ftcPose.bearing;

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -0.4,0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        // Drive
        drive.drive(forward,strafe,rotate);

        // Tuner
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }

        // Telemetry
        if (id20 != null) {
            if (gamepad1.left_trigger > 0.5) {
                telemetry.addLine("Auto Align");
            }
            webcam.displayDetectionTelemetry(id20);
            telemetry.addData("Error", error);
        } else {
            telemetry.addLine("Manual Rotate Mode");
        }
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad L/R)", kP);
        telemetry.addData("Tuning D","%.4f (D-Pad U/D)", kD);
        telemetry.addData("Step Size","%.4f (B Button)", stepSizes[stepIndex]);

    }
}
