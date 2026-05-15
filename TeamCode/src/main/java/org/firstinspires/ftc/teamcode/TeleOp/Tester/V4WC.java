package org.firstinspires.ftc.teamcode.TeleOp.Tester;

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

//@TeleOp(name = "V4-WC", group = "Tester")
public class V4WC extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Follower follower;
    ShooterV2 shooter = new ShooterV2();
    Turret turret = new Turret();
    private Limelight3A limelight;

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
    boolean turretOn = true;
    double turretPower = 0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseConstant.BlueAutoStartPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(2); // BlueTag

        drive.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);

        timer.reset();

        telemetry.addLine("Ready to start");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        turret.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        LLResult llResult = limelight.getLatestResult();

        if (gamepad2.xWasPressed()) {
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
        updateHoodMode();
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
            LLResult result = limelight.getLatestResult();
            TurretAim(result);
            drive.drive(forward,strafe,rotate);
        }

        // SUBSYSTEMS
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
        telemetry.addLine("-------------- Boolean ------------");
        telemetry.addData("CameraTagDetected?", llResult.isValid());
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
        limelight.stop();
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

        // Shooter
        if (gamepad1.right_bumper) {   //&& turret.isAligned(follower.getHeading()
            shooter.fireShot();
        }

        // Hood
        shooter.setHood(HoodPos);

    }

    public void TurretAim(LLResult result) {
        if (turretOn) {
            Pose robotPose = follower.getPose();
            double dt = timer.seconds();
            timer.reset();
            if (result.isValid()) {
                // ===== CAMERA AIM =====
                double turretAngle = Math.toRadians(turret.getCurrentAngle());
                double txRad = Math.toRadians(result.getTx());
                double targetTurretAngle = turretAngle - txRad;
                targetTurretAngle = Range.clip(targetTurretAngle, TurretConstant.MIN_ANGLE, TurretConstant.MAX_ANGLE);
                double error = Util.angleWrap(targetTurretAngle - turretAngle);
                if (Math.abs(error) < TurretConstant.angleTolerance) {
                    turretPower = 0;
                    lastErrorCam = 0;
                } else {
                    double pTerm = error * TurretConstant.CamkP;
                    double dTerm = 0;
                    if (dt > 0) {
                        dTerm = ((error - lastErrorCam) / dt) * TurretConstant.CamkD;
                    }
                    lastErrorCam = error;
                    double FeedForward = TurretConstant.CamkF * follower.getAngularVelocity();
                    turretPower = pTerm + dTerm - FeedForward;
                }
                turret.setPower(Range.clip(turretPower,-TurretConstant.MAX_POWER,TurretConstant.MAX_POWER));

            } else {
                // ===== ODOMETRY AIM =====
                double targetAngle = drive.getAngleToGoal(robotPose, GOAL);
                double robotHeading = robotPose.getHeading();
                double robotTarget = Util.angleWrap(targetAngle - robotHeading);
                double turretAngle = Math.toRadians(turret.getCurrentAngle());
                double targetTurretAngle = robotTarget;
                targetTurretAngle = Range.clip(targetTurretAngle, Math.toRadians(TurretConstant.MIN_ANGLE), Math.toRadians(TurretConstant.MAX_ANGLE));
                double error = Util.angleWrap(targetTurretAngle - turretAngle);
                if (Math.abs(error) < TurretConstant.angleTolerance) {
                    turretPower = 0;
                    lastErrorOdo = 0;
                } else {
                    double pTerm = error * TurretConstant.OdokP;
                    double dTerm = 0;
                    if (dt > 0) {
                        dTerm = ((error - lastErrorOdo) / dt) * TurretConstant.OdokD;
                    }
                    lastErrorOdo = error;
                    double FeedForward = TurretConstant.OdokF * follower.getAngularVelocity();
                    turretPower = pTerm + dTerm - FeedForward;
                }
                turret.setPower(Range.clip(turretPower,-TurretConstant.MAX_POWER,TurretConstant.MAX_POWER));
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
                HoodPos += 0.05;
            }
            if (gamepad2.dpad_down) {
                HoodPos -= 0.05;
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
