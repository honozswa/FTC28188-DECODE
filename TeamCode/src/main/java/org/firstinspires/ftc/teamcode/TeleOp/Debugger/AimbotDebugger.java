package org.firstinspires.ftc.teamcode.TeleOp.Debugger;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Util;

@TeleOp(name = "Debugger-FusionAimbot", group = "Debugger")
public class AimbotDebugger extends OpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private Limelight3A limelight;
    Follower follower;

    // Aiming PID Variables
    double lastErrorLL = 0;
    double lastErrorOdo = 0;
    private static final Pose GOAL = PoseConstant.RED_GOAL;
    private final ElapsedTime timer = new ElapsedTime();

    // ---------------------- driving setup --------------------------------- //
    double forward,strafe,rotate;

    @Override
    public void init() {

        drive.init(hardwareMap);

    }

    @Override
    public void loop() {
        // Mecanum Drive Input
        forward =- gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // Auto Align Logic
        Aimbot(limelight.getLatestResult());

        // Drive
        drive.drive(forward,strafe,rotate);

        // Telemetry

        if (gamepad1.left_trigger > 0.5) {
            telemetry.addLine("Auto Align");
        }
        else {
            telemetry.addLine("Manual Rotate Mode");
        }

    }

    public void Aimbot(LLResult result) {
        if (gamepad1.left_trigger > 0.5) {
            double dt = timer.seconds();
            timer.reset();
            if (result.isValid()) {
                // ===== CAMERA AIM =====
                double error = result.getTx() + ShooterConstant.llOffset;
                double pTerm = error * ShooterConstant.llkP;
                double dTerm = 0;
                if (dt > 0) { dTerm = ((error - lastErrorLL) / dt) * ShooterConstant.llkD;}
                double rawRotate = Range.clip(pTerm + dTerm, -ShooterConstant.maxRotatePower, ShooterConstant.maxRotatePower);
                if (Math.abs(rawRotate) < ShooterConstant.lowPowerThreshold) {
                    rotate = Math.signum(rawRotate) * Math.pow(Math.abs(rawRotate), ShooterConstant.exponent);
                } else {
                    rotate = rawRotate;
                }
                lastErrorLL = error;
                telemetry.addLine("Camera in use");
                telemetry.addData("error", error);
                telemetry.addData("rotate", rotate);
                telemetry.addLine("");
            } else {
                Pose robotPose = follower.getPose();
                double targetHeading = drive.getAngleToGoal(robotPose, GOAL);
                double error = Util.angleWrap(robotPose.getHeading() - targetHeading + ShooterConstant.odoOffset);
                double pTerm = error * ShooterConstant.odokP;
                double dTerm = 0;
                if (dt > 0) { dTerm = ((error - lastErrorOdo) / dt) * ShooterConstant.odokD;}
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

}
