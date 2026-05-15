package org.firstinspires.ftc.teamcode.TeleOp.OutOfDated;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.CameraConstant;
import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.Constants.TurretConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.ShooterV2;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.mechanism.Util;
import org.firstinspires.ftc.teamcode.mechanism.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//@TeleOp(name = "Blue-V4-Webcam")
public class V4ControlWebcam extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();
    Follower follower;
    ShooterV2 shooter = new ShooterV2();
    Turret turret = new Turret();
    private final Webcam webcam = new Webcam();

    // Drivetrain
    double forward = 0, strafe = 0, rotate = 0;

    // Flywheel Vel
    double targetVelocity = 0;
    boolean autoShooterEnabled = false;
    double ZERO_VELOCITY = ShooterConstant.ZeroVel;
    double CLOSE_VELOCITY = ShooterConstant.CloseVel;
    final double VELOCITY_STEP = ShooterConstant.VelStep;

    // Intake Outtake
    private boolean intakeOn = false;

    // Hood
    double HoodPos = ShooterConstant.minServoPos2;
    boolean autoHoodEnabled = false;
    enum ShooterMode { CLOSE, MID, FAR }
    ShooterMode currentMode = ShooterMode.CLOSE;

    // Pose
    private static final Pose GOAL = PoseConstant.BLUE_GOAL;
    boolean autoDriving = false;
    double filteredDistance = 36;

    // Camera Variables
    double lastErrorCam = 0;
    double lastErrorOdo = 0;

    // Turret
    boolean turretOn = false;
    double turretPower = 0;
    private final ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseConstant.BlueAutoStartPose);

        webcam.init(hardwareMap, telemetry);
        drive.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);

        timer.reset();

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            webcam.update();
            AprilTagDetection id = webcam.getTagBySpecificId(CameraConstant.BlueAprilTagsId);

            if (gamepad2.xWasPressed()) {
                autoShooterEnabled = !autoShooterEnabled;
            }
            if (gamepad2.yWasPressed()) {
                autoHoodEnabled = !autoHoodEnabled;
            }
            if (gamepad2.aWasPressed()) {
                turretOn = !turretOn;
            }
            if (gamepad2.bWasPressed()) {
                turret.resetEncoder();
            }

            updateDistance();
            updateHoodMode();
            autoFlywheel();
            autoHood();

            /* =====================
               AUTO DRIVE
            ===================== */

            if(gamepad1.left_bumper && !autoDriving) {
                goToShootPose();
                autoDriving = true;
            }

            /* =====================
               DRIVER OVERRIDE
            ===================== */

            if(autoDriving && drive.driverOverride()) {
                follower.breakFollowing();
                drive.stopDrive();
                drive.setBrake();
                autoDriving = false;
            }

            /* =====================
               DRIVE CONTROL
            ===================== */

            if(autoDriving) {
                follower.update();
                if(!follower.isBusy()) {
                    follower.breakFollowing();
                    drive.stopDrive();
                    autoDriving = false;
                    drive.setBrake();
                }
            } else {
                manualDrive();
//                FusionAim(id);
                TurretAim(id);
                drive.drive(forward,strafe,rotate);
            }

            /* =====================
               SUBSYSTEMS
            ===================== */

            subSystem();
            shooter.update();

            // =======================
            // Telemetry
            // =======================

            telemetry.addData("Shooter1 Power:", shooter.getFlywheelPower1());
            telemetry.addData("Intake Power:", shooter.getIntakePower());
            telemetry.addData("Target Vel:", targetVelocity);
            telemetry.addData("Vel1:", shooter.getFlywheelVel1());
            telemetry.addData("Hood", shooter.getHoodPos());
            telemetry.addData("Distance", drive.getDistanceToGoal(follower.getPose(),GOAL));
            telemetry.update();
        }
    }

    public void goToShootPose() {

        PathChain shootPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        PoseConstant.BLUE_SHOOT_POSE
                ))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        PoseConstant.BLUE_SHOOT_POSE.getHeading()
                )
                .build();

        follower.followPath(shootPath);
    }

    public void manualDrive() {
        strafe = gamepad1.left_stick_x;
        forward = -gamepad1.left_stick_y;
        rotate = gamepad1.right_stick_x / 1.8;
    }

    public void subSystem() {
        // =======================
        // Flywheel
        // =======================

        shooter.flywheelOn(targetVelocity);

        // =======================
        // Intake
        // =======================

        if (gamepad1.aWasPressed()) {
            intakeOn = !intakeOn;
        }

        if (intakeOn) {
            shooter.intakeOn();
        } else {
            shooter.intakeOff();
        }

        shooter.setIntakeBoost(gamepad1.right_trigger > 0.5);

        // =======================
        // Shooter
        // =======================

        if (gamepad1.right_bumper) {   //&& turret.isAligned(follower.getHeading()
            shooter.fireShot();
        }

        // =======================
        // Hood
        // =======================

        shooter.setHood(HoodPos);

    }

//    public void FusionAim(AprilTagDetection id) {
//        if (gamepad1.left_trigger > 0.5) {
//            if (id != null) {
//                CamError = CameraConstant.goalX - id.ftcPose.bearing;
//
//                if (Math.abs(CamError) < CameraConstant.angleTolerance) {
//                    rotate = 0;
//                } else {
//                    double pTerm = CamError * CameraConstant.kP;
//
//                    curTime = getRuntime();
//                    double dT = curTime - lastTime;
//                    double dTerm = ((CamError - lastCamError) / dT) * CameraConstant.kD;
//
//                    rotate = Range.clip(pTerm + dTerm, -0.4,0.4);
//
//                    lastCamError = CamError;
//                    lastTime = curTime;
//                }
//            } else {
//                Pose robotPose = follower.getPose();
//                double targetHeading = drive.getAngleToGoal(robotPose,GOAL);
//                double error = Util.angleWrap(robotPose.getHeading() - targetHeading);
//                double kP = 1;
//                rotate = Range.clip(error * kP, -1, 1);
//            }
//        }
//    }

    public void TurretAim(AprilTagDetection id) {
        if (turretOn) {
            Pose robotPose = follower.getPose();
//            LLResult result = limelight.getLatestResult();
            double dt = timer.seconds();
            timer.reset();
            if (id != null) {
                // ===== CAMERA AIM =====
                double error = -id.ftcPose.bearing;
                if (Math.abs(error) < TurretConstant.angleTolerance) {
                    turretPower = 0;
                    lastErrorCam = 0;
                } else {
                    double pTerm = error * TurretConstant.CamkP;
                    double dTerm = 0;
                    if (dt > 0) {
                        dTerm = ((error - lastErrorCam) / dt) * TurretConstant.CamkD;
                    }
                    double FeedForward = TurretConstant.CamkF * Math.toDegrees(follower.getAngularVelocity());
                    turretPower = Range.clip(pTerm + dTerm + FeedForward, -TurretConstant.MAX_POWER,TurretConstant.MAX_POWER);
                    turret.setPower(turretPower);

                    lastErrorCam = error;
                }
            } else {
                // ===== ODOMETRY AIM =====
                double targetAngle = Math.toDegrees(drive.getAngleToGoal(robotPose, GOAL));
                double RobotTarget = Util.angleWrap(targetAngle - Math.toDegrees(follower.getHeading()));
                double error = Util.angleWrap(RobotTarget - turret.getCurrentAngle());
                if (Math.abs(error) < TurretConstant.angleTolerance) {
                    turretPower = 0;
                    lastErrorOdo = 0;
                } else {
                    double pTerm = error * TurretConstant.OdokP;
                    double dTerm = 0;
                    if (dt > 0) {
                        dTerm = ((error - lastErrorOdo) / dt) * TurretConstant.OdokD;
                    }
                    double FeedForward = TurretConstant.OdokF * Math.toDegrees(follower.getAngularVelocity());
                    turretPower = Range.clip(pTerm + dTerm + FeedForward, -TurretConstant.MAX_POWER,TurretConstant.MAX_POWER);
                    turret.setPower(turretPower);

                    lastErrorOdo = error;
                }
            }
        } else {
            turret.setPower(gamepad2.left_stick_x * 0.5);
        }
    }

    public void updateDistance() {
        Pose robotPose = follower.getPose();
        double rawDistance = drive.getDistanceToGoal(robotPose, GOAL);
        filteredDistance = 0.8 * filteredDistance + 0.2 * rawDistance;
    }

    public void updateHoodMode() {
        double d = filteredDistance;

        switch (currentMode) {
            case CLOSE:
                if (d > ShooterConstant.closeRange)
                    currentMode = ShooterMode.MID;
                break;

            case MID:
                if (d < ShooterConstant.closeRange)
                    currentMode = ShooterMode.CLOSE;
                else if (d > ShooterConstant.midRange)
                    currentMode = ShooterMode.FAR;
                break;

            case FAR:
                if (d < ShooterConstant.midRange)
                    currentMode = ShooterMode.MID;
                break;
        }
    }

    public void autoFlywheel() {

        if (!autoShooterEnabled) {

            if (gamepad1.yWasPressed()) {
                targetVelocity = CLOSE_VELOCITY;
            }
            if (gamepad1.bWasPressed()) {
                targetVelocity = ZERO_VELOCITY;
            }
            if (gamepad1.dpadUpWasPressed()) {
                targetVelocity += VELOCITY_STEP;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetVelocity -= VELOCITY_STEP;
            }

            targetVelocity = Math.max(0, targetVelocity);

        } else {

            double distance = filteredDistance;

            if (currentMode == ShooterMode.CLOSE) {
                double velocity = Range.clip(Util.getFlywheelVelocityFromDistanceClose(distance), ShooterConstant.minTicks, ShooterConstant.maxTicks);
                targetVelocity = 0.8 * targetVelocity + 0.2 * velocity;
            }
            else if (currentMode == ShooterMode.MID) {
                double velocity = Range.clip(Util.getFlywheelVelocityFromDistanceMid(distance), ShooterConstant.minTicks, ShooterConstant.maxTicks);
                targetVelocity = 0.8 * targetVelocity + 0.2 * velocity;
            }
            else {
                double velocity = Range.clip(Util.getFlywheelVelocityFromDistanceFar(distance), ShooterConstant.minTicks, ShooterConstant.maxTicks);
                targetVelocity = 0.8 * targetVelocity + 0.2 * velocity;
            }

        }
    }

    public void autoHood() {

        if (!autoHoodEnabled) {

            if (gamepad2.dpad_up) {
                HoodPos += 0.01;
            }
            if (gamepad2.dpad_down) {
                HoodPos -= 0.01;
            }
            HoodPos = Range.clip(HoodPos, ShooterConstant.minServoPos2, ShooterConstant.maxServoPos1);

        } else {

            if (currentMode == ShooterMode.CLOSE) {
                double target = ShooterConstant.CloseModePos;
                HoodPos = 0.8 * HoodPos + 0.2 * target;
            }
            else if (currentMode == ShooterMode.MID) {
                double target = ShooterConstant.MidModePos;
                HoodPos = 0.8 * HoodPos + 0.2 * target;
            }
            else {
                double target = ShooterConstant.FarModePos;
                HoodPos = 0.8 * HoodPos + 0.2 * target;
            }

        }
    }

}
