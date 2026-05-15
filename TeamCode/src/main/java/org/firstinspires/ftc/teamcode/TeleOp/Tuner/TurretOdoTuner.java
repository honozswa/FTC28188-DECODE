package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.TurretConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.mechanism.Util;

@TeleOp(name = "Tuner-TurretOdo", group = "Tuner")
public class TurretOdoTuner extends OpMode {

//    private Webcam webcam = new Webcam();
//    private Limelight3A limelight;
    private Turret turret = new Turret();
    private MecanumDrive drive = new MecanumDrive();
    Follower follower;
    boolean turretOn = true;

    double OdoF = TurretConstant.OdokF;
    double OdoP = TurretConstant.OdokP;
    double OdoD = TurretConstant.OdokD;
    
    // Camera Variables
    double lastErrorCam = 0;
    double lastErrorOdo = 0;

    // Turret
    double turretPower = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // Pose
    private Pose GOAL = PoseConstant.RED_GOAL;

    // ---------------------- driving setup --------------------------------- //
    double forward,strafe,rotate;

    // --------------------- controller based PD tuning ----------------------- //
    double[] stepSizes = {1,0.1,0.01,0.001,0.0001,0.00001};
    int stepIndex = 1;

    @Override
    public void init() {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseConstant.RedCloseAutoStartPose);

//        webcam.init(hardwareMap, telemetry);
//        limelight = hardwareMap.get(Limelight3A.class,"limelight");
//        limelight.pipelineSwitch(4); // BlueTag
//        limelight.start();
//        sleep(1000);

        turret.init(hardwareMap);
        drive.init(hardwareMap);

        timer.reset();

//        if (limelight.isConnected()) {;
//            telemetry.addLine("Limelight Connected");
//            telemetry.addLine("Starting...");
//            telemetry.update();
//            if (limelight.isRunning()) {
//                telemetry.addLine("Ready to start");
//                telemetry.update();
//            }
//        } else {
//            telemetry.addLine("Camera is not Connected");
//            if (!limelight.isConnected()) {
//                limelight.stop();
//                sleep(1000);
//                limelight.start();
//            }
//            telemetry.update();
//        }

        telemetry.addLine("Ready to start");
        telemetry.update();
    }

    @Override
    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();

        // Mecanum Drive Input
        forward =- gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

//        if (gamepad1.xWasPressed()) {
//            useCamera = !useCamera;
//        }

//        webcam.update();
//        AprilTagDetection id20 = webcam.getTagBySpecificId(20);
//        LLResult result = limelight.getLatestResult();
        TurretAim();
//        if (turretOn) {
//            if (useCamera) {
//                TurretAim();
//            } else {
//                turret.setTargetAngle(curAngle);
//            }
//        }

//        if (!useCamera) {
//            drive.drive(0,0,0);
//        } else {
//            drive.drive(forward, strafe, rotate);
//        }

        if (gamepad1.leftBumperWasPressed()) {
            turret.resetEncoder();
        }

//        turret.update(follower);

        // Drive
        drive.drive(forward,strafe,rotate);

        // Tuner
//        if (gamepad1.yWasPressed()) {
//            if (curAngle == centerAngle) {
//                curAngle = offAngle;
//            } else { curAngle = centerAngle; }
//        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.aWasPressed()) {
            turretOn = !turretOn;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            OdoF -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            OdoF += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            OdoP += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            OdoP -= stepSizes[stepIndex];
        }

        if (gamepad2.dpadUpWasPressed()) {
            OdoD += stepSizes[stepIndex];
        }

        if (gamepad2.dpadDownWasPressed()) {
            OdoD -= stepSizes[stepIndex];
        }


//        if (result.isValid()) {
//            telemetry.addLine("Detected");
//        } else {
//            telemetry.addLine("No Tag Detected");
//        }

        telemetry.addLine("--------------------------");
        telemetry.addLine("leftBumper for turret heading reset");
        telemetry.addData("TurretOn", turretOn);
//        telemetry.addData("Camera in use (x)", useCamera);
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)", OdoF);
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)", OdoP);
        telemetry.addData("Tuning D","%.4f (D-Pad2 U/D)", OdoD);
        telemetry.addData("Step Size","%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.addLine("-------------- Pose ------------");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Distance", drive.getDistanceToGoal(follower.getPose(),GOAL));
        telemetry.addLine("");
//        telemetry.addLine("-------------- Camera ------------");
//        telemetry.addData("CameraTagDetected?", limelight.getLatestResult().isValid());
//        telemetry.addData("PipelineIndex", limelight.getStatus().getPipelineIndex());
//        telemetry.addData("FPS", limelight.getStatus().getFps());
//        telemetry.addData("CPU",limelight.getStatus().getCpu());
//        telemetry.addData("RAM",limelight.getStatus().getRam());
//        telemetry.addData("Temp",limelight.getStatus().getTemp());
//        telemetry.addData("Connected", limelight.isConnected());
//        telemetry.addData("Running", limelight.isRunning());
//        telemetry.addLine("");

    }

    public void TurretAim() {
        if (turretOn) {
            Pose robotPose = follower.getPose();
            double dt = timer.seconds();
            timer.reset();
            // ===== ODOMETRY AIM =====
            double targetAngle = drive.getAngleToGoal(robotPose, GOAL);
            double robotHeading = robotPose.getHeading();
            double robotBackwardHeading = Util.angleWrap(robotHeading + Math.PI);
            double robotTarget = Util.angleWrap(targetAngle - robotBackwardHeading);
            double clippedRobotTarget = Range.clip(robotTarget, TurretConstant.MIN_ANGLE, TurretConstant.MAX_ANGLE);
            double turretAngle = Math.toRadians(turret.getCurrentAngle());
            double error = clippedRobotTarget - turretAngle;
            double pTerm = error * OdoP;
            double dTerm = 0;
            if (dt > 0) { dTerm = ((error - lastErrorOdo) / dt) * OdoD; }
            lastErrorOdo = error;
            double rotationalFF = OdoF * follower.getAngularVelocity();
            turretPower = pTerm + dTerm - rotationalFF;

            turret.setPower(Range.clip(turretPower,-TurretConstant.MAX_POWER,TurretConstant.MAX_POWER));

            telemetry.addLine("Odometry in use");
            telemetry.addData("targetAngle(atan2)", Math.toDegrees(targetAngle));
            telemetry.addData("robotHeading", Math.toDegrees(robotHeading));
            telemetry.addData("robotTarget", Math.toDegrees(robotTarget));
            telemetry.addData("turretTicks", turret.getTicks());
            telemetry.addData("turretAngle", Math.toDegrees(turretAngle));
            telemetry.addData("error", error);
            telemetry.addData("turretPower", turretPower);
            telemetry.addLine("");


        } else {
            turret.setPower(gamepad2.left_stick_x * 0.5 );
        }
    }

}
