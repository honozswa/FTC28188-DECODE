package org.firstinspires.ftc.teamcode.TeleOp.Main;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.Constants.TurretConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.ShooterV2;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.mechanism.Util;

@TeleOp(name = "Red-V4-OpMode", group = "Main")
public class RedV4OpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Follower follower;
    ShooterV2 shooter = new ShooterV2();
    Turret turret = new Turret();
    private Limelight3A limelight;

    // Drivetrain
    double forward = 0, strafe = 0, rotate = 0;

    // Flywheel Vel
    double targetVelocity = 0;
    boolean autoShooterEnabled = true;
    double ZERO_VELOCITY = ShooterConstant.ZeroVel;
    double CLOSE_VELOCITY = ShooterConstant.CloseVel;
    double FarVel = ShooterConstant.FarVel;
    final double VELOCITY_STEP = ShooterConstant.VelStep;

    // Intake Outtake
    private boolean intakeOn = false;

    // Hood
    double HoodPos = ShooterConstant.minServoPos2;
    boolean autoHoodEnabled = true;

    // Pose
    private static final Pose GOAL = PoseConstant.RED_GOAL;
    boolean autoDriving = false;
    double filteredDistance = 36;

    // Camera Variables
    double lastErrorCam = 0;
    double lastErrorOdo = 0;

    // Turret
    boolean turretOn = true;
    double turretPower = 0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

//        if (PoseConstant.hasAutoPose) {
//            follower.setStartingPose(PoseConstant.AutoEndPose);
//            PoseConstant.hasAutoPose = false;
//        } else {
        follower.setStartingPose(PoseConstant.RedCloseAutoStartPose);
//        }

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4); // RedTag
        limelight.start();
//        sleep(1000);

        drive.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);
//        turret.setAngleOffset(TurretConstant.TurretAngleOffset);

        timer.reset();
    }

    @Override
    public void init_loop() {
        if (limelight.isConnected()) {;
            telemetry.addLine("Limelight Connected");
            telemetry.addLine("Starting...");
            telemetry.update();
            if (limelight.isRunning()) {
                telemetry.addLine("Ready to start");
                telemetry.update();
            }
        } else {
            telemetry.addLine("Camera is not Connected");
            if (!limelight.isConnected()) {
                limelight.stop();
                sleep(1000);
                limelight.start();
            }
            telemetry.update();
        }
    }

    @Override
    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        if (gamepad1.x || gamepad2.b) {
            shooter.outtakeOn();
        } else {
            shooter.outtakeOff();
        }

        if (gamepad2.xWasPressed()) {
            targetVelocity = 0;
            autoShooterEnabled = !autoShooterEnabled;
        }
        if (gamepad2.yWasPressed()) {
            autoHoodEnabled = !autoHoodEnabled;
        }
        if (gamepad2.aWasPressed()) {
            turretOn = !turretOn;
        }
        if (gamepad2.leftBumperWasPressed()) {
            turret.resetEncoder();
        }

        updateDistance();
//        updateHoodMode();
        autoFlywheel();
        autoHood();

        // AUTO DRIVE
        if(gamepad1.left_bumper && !autoDriving) {
            goToShootPose();
            autoDriving = true;
        }

        // DRIVER OVERRIDE
        if(autoDriving && drive.driverOverride()) {
            follower.breakFollowing();
            drive.stopDrive();
            drive.setBrake();
            autoDriving = false;
        }

        // DRIVE CONTROL
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
            drive.drive(forward,strafe,rotate);
        }

        // SUBSYSTEMS
        TurretAim(limelight.getLatestResult());
        subSystem();
        shooter.update();

        // Telemetry
        telemetry.addLine("-------------- Shooter ------------");
        telemetry.addData("Target Vel:", targetVelocity);
        telemetry.addData("Current Vel:", shooter.getFlywheelVel1());
        telemetry.addData("Hood", shooter.getHoodPos());
        telemetry.addData("Shooter Power:", shooter.getFlywheelPower1());
        telemetry.addData("Intake Power:", shooter.getIntakePower());
        telemetry.addLine("");
        telemetry.addLine("-------------- Pose ------------");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Distance", drive.getDistanceToGoal(follower.getPose(),GOAL));
        telemetry.addLine("");
        telemetry.addLine("-------------- Camera ------------");
        telemetry.addData("CameraTagDetected?", limelight.getLatestResult().isValid());
        telemetry.addData("PipelineIndex", limelight.getStatus().getPipelineIndex());
        telemetry.addData("FPS", limelight.getStatus().getFps());
        telemetry.addData("CPU",limelight.getStatus().getCpu());
        telemetry.addData("RAM",limelight.getStatus().getRam());
        telemetry.addData("Temp",limelight.getStatus().getTemp());
        telemetry.addData("Connected", limelight.isConnected());
        telemetry.addData("Running", limelight.isRunning());
        telemetry.addLine("");
        telemetry.addLine("-------------- Boolean ------------");
        telemetry.addData("TurretOn", turretOn);
        telemetry.addData("ShooterOn", autoShooterEnabled);
        telemetry.addData("HoodOn", autoHoodEnabled);
        telemetry.addLine("");
        telemetry.addLine("-------------- Gamepad1 ------------");
        telemetry.addLine("MecanumDrive: left/right stick");
        telemetry.addLine("ToggleIntake: A");
        telemetry.addLine("IntakeSpeedUp: RT");
        telemetry.addLine("Shot: RB");
        telemetry.addLine("FlywheelCloseVel: Y");
        telemetry.addLine("FlywheelOff: B");
        telemetry.addLine("AdjustFlywheelVel: Dpad Up/Down");
        telemetry.addLine("AutoPark: LB");
        telemetry.addLine("");
        telemetry.addLine("-------------- Gamepad2 ------------");
        telemetry.addLine("AutoFlywheel: X");
        telemetry.addLine("AutoHood: Y");
        telemetry.addLine("AutoTurret: A");
        telemetry.addLine("ResetTurret: LB");
        telemetry.addLine("AdjustHood: Dpad Up/Down");
        telemetry.addLine("IntakeSpeedUp: RT");
        telemetry.addLine("TurnTurret: leftStick_X");
        telemetry.addLine("");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.shutdown();
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
        // Flywheel
        shooter.flywheelOn(targetVelocity);

        // Intake
        if (gamepad1.aWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (intakeOn) {
            shooter.intakeOn();
        } else {
            shooter.intakeOff();
        }
        shooter.setIntakeBoost(gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5);

        if (gamepad2.dpadLeftWasPressed()) {
            shooter.increaseIntakeOffset();
        } else if (gamepad2.dpadRightWasPressed()) {
            shooter.decreaseIntakeOffset();
        }

        // Shooter
        if (gamepad1.rightBumperWasPressed()) {   //&& turret.isAligned(follower.getHeading()
            shooter.fireShot();
        }
        if (gamepad1.left_trigger > 0.5) {
            shooter.setReturnRequested();
        }

        // Hood
        shooter.setHood(HoodPos);

    }

    public void TurretAim(LLResult result) {
        if (turretOn) {
            Pose robotPose = follower.getPose();
            double dt = timer.seconds();
            timer.reset();
            if ((result.isValid()) && drive.getDistanceToGoal(robotPose, GOAL) > 105) {
                // ===== CAMERA AIM =====
                double turretAngle = turret.getCurrentAngle();
                double txRad = result.getTx();
                double error = -txRad + TurretConstant.CamOffset;
                boolean atMin = turretAngle <= Math.toDegrees(TurretConstant.MIN_ANGLE);
                boolean atMax = turretAngle >= Math.toDegrees(TurretConstant.MAX_ANGLE);
                if ((atMin && error < 0) || (atMax && error > 0)) { error = 0;}
                double pTerm = error * TurretConstant.CamkP;
                double dTerm = 0;
                if (dt > 0) { dTerm = ((error - lastErrorCam) / dt) * TurretConstant.CamkD;}
                lastErrorCam = error;
                double FeedForward = TurretConstant.CamkF * follower.getAngularVelocity();
                double rawPower = pTerm + dTerm - FeedForward;

                if (Math.abs(rawPower) < TurretConstant.lowPowerThreshold) {
                    turretPower = Math.signum(rawPower) * Math.pow(Math.abs(rawPower), TurretConstant.exponent);
                } else {
                    turretPower = rawPower;
                }
                turret.setPower(Range.clip(turretPower,-TurretConstant.MAX_POWER,TurretConstant.MAX_POWER));

                telemetry.addLine("Camera in use");
//                telemetry.addData("turretTicks", turret.getTicks());
//                telemetry.addData("turretAngle", Math.toDegrees(turretAngle));
//                telemetry.addData("tx", Math.toDegrees(txRad));
//                telemetry.addData("error", error);
//                telemetry.addData("turretPower", turretPower);
                telemetry.addLine("");

            } else {
                // ===== ODOMETRY AIM =====
                double targetAngle = drive.getAngleToGoal(robotPose, GOAL);
                double robotHeading = robotPose.getHeading();
                double robotBackwardHeading = Util.angleWrap(robotHeading + Math.PI);
                double robotTarget = Util.angleWrap(targetAngle - robotBackwardHeading);
                double clippedRobotTarget = Range.clip(robotTarget, TurretConstant.MIN_ANGLE, TurretConstant.MAX_ANGLE);
                double turretAngle = Math.toRadians(turret.getCurrentAngle());
                double error = clippedRobotTarget - turretAngle;
                double pTerm = error * TurretConstant.OdokP;
                double dTerm = 0;
                if (dt > 0) { dTerm = ((error - lastErrorOdo) / dt) * TurretConstant.OdokD; }
                lastErrorOdo = error;
                double rotationalFF = TurretConstant.OdokF * follower.getAngularVelocity();
                turretPower = pTerm + dTerm - rotationalFF;
                if (Math.abs(turretPower) < TurretConstant.lowPowerThreshold) {
                    turretPower = Math.signum(turretPower) * Math.pow(Math.abs(turretPower), TurretConstant.exponent);
                }
                turret.setPower(Range.clip(turretPower,-TurretConstant.MAX_POWER,TurretConstant.MAX_POWER));

                telemetry.addLine("Odometry in use");
//                telemetry.addData("targetAngle(atan2)", Math.toDegrees(targetAngle));
//                telemetry.addData("robotHeading", Math.toDegrees(robotHeading));
//                telemetry.addData("robotTarget", Math.toDegrees(robotTarget));
//                telemetry.addData("turretTicks", turret.getTicks());
//                telemetry.addData("turretAngle", Math.toDegrees(turretAngle));
//                telemetry.addData("error", error);
//                telemetry.addData("turretPower", turretPower);
                telemetry.addLine("");

            }
        } else {
            turret.setPower(gamepad2.left_stick_x * 0.5 );
        }
    }

    public void updateDistance() {
        Pose robotPose = follower.getPose();
        double rawDistance = drive.getDistanceToGoal(robotPose, GOAL);
        filteredDistance = 0.8 * filteredDistance + 0.2 * rawDistance;
    }

//    public void updateHoodMode() {
//        double d = filteredDistance;
//
//        switch (currentMode) {
//            case CLOSE:
//                if (d > ShooterConstant.closeRange)
//                    currentMode = ShooterMode.MID;
//                break;
//
//            case MID:
//                if (d < ShooterConstant.closeRange)
//                    currentMode = ShooterMode.CLOSE;
//                else if (d > ShooterConstant.midRange)
//                    currentMode = ShooterMode.FAR;
//                break;
//
//            case FAR:
//                if (d < ShooterConstant.midRange)
//                    currentMode = ShooterMode.MID;
//                break;
//        }
//    }

    public void autoFlywheel() {

        if (!autoShooterEnabled) {

            if (gamepad1.yWasPressed()) {
                targetVelocity = FarVel;
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

//            if (currentMode == ShooterMode.CLOSE) {
//                double velocity = Range.clip(Util.getFlywheelVelocityFromDistanceClose(distance), ShooterConstant.minTicks, ShooterConstant.maxTicks);
//                targetVelocity = 0.8 * targetVelocity + 0.2 * velocity;
//            }
//            else if (currentMode == ShooterMode.MID) {
//                double velocity = Range.clip(Util.getFlywheelVelocityFromDistanceMid(distance), ShooterConstant.minTicks, ShooterConstant.maxTicks);
//                targetVelocity = 0.8 * targetVelocity + 0.2 * velocity;
//            }
//            else {
//                double velocity = Range.clip(Util.getFlywheelVelocityFromDistanceFar(distance), ShooterConstant.minTicks, ShooterConstant.maxTicks);
//                targetVelocity = 0.8 * targetVelocity + 0.2 * velocity;
//            }

            targetVelocity = Util.getFlywheelVelocityFromDistance(distance);

        }
    }

    public void autoHood() {

        if (!autoHoodEnabled) {

            if (gamepad2.dpad_up) {
                HoodPos += 0.05;
            }
            if (gamepad2.dpad_down) {
                HoodPos -= 0.05;
            }
            HoodPos = Range.clip(HoodPos, ShooterConstant.minServoPos2, ShooterConstant.maxServoPos1);

        } else {

            double distance = filteredDistance;

//            if (currentMode == ShooterMode.CLOSE) {
//                double target = ShooterConstant.CloseModePos;
//                HoodPos = 0.8 * HoodPos + 0.2 * target;
//            }
//            else if (currentMode == ShooterMode.MID) {
//                double target = ShooterConstant.MidModePos;
//                HoodPos = 0.8 * HoodPos + 0.2 * target;
//            }
//            else {
//                double target = ShooterConstant.FarModePos;
//                HoodPos = 0.8 * HoodPos + 0.2 * target;
//            }

            HoodPos = Util.getHoodPositionFromDistance(distance);

        }
    }

}