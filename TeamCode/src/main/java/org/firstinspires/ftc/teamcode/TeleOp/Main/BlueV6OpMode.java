package org.firstinspires.ftc.teamcode.TeleOp.Main;

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
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.ShooterV6;
import org.firstinspires.ftc.teamcode.mechanism.Util;

@TeleOp(name = "Blue-V6-OpMode", group = "Main")
public class BlueV6OpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Follower follower;
    ShooterV6 shooter = new ShooterV6();
    private Limelight3A limelight;

    // Drivetrain
    double forward = 0, strafe = 0, rotate = 0;
    double curForward = 0, curStrafe = 0;

    // Flywheel
    double targetVelocity = 0;
    boolean autoShooterEnabled = true;
    double ZeroVel = ShooterConstant.ZeroVel;
    double FarVel = ShooterConstant.FarVel;
    final double VelStep = ShooterConstant.VelStep;
    private double FlywheelManualOffset = 0;

    // Intake
    private boolean intakeOn = true;
    private boolean lastFull = false;

    // Hood
    double HoodPos = ShooterConstant.minHoodPos;
    boolean autoHoodEnabled = true;
    private double HoodOffset = 0;

    // Pose
    private static final Pose GOAL = PoseConstant.BLUE_GOAL;
    boolean autoDriving = false;
    double filteredDistance = 36;

    // Aiming PID Variables
    double lastErrorLL = 0;
    double lastErrorOdo = 0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        if (PoseConstant.hasAutoPose) {
            follower.setStartingPose(PoseConstant.AutoEndPose);
            PoseConstant.hasAutoPose = false;
        } else {
            follower.setStartingPose(PoseConstant.BlueCloseAutoStartPose);
        }

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(ShooterConstant.blueTagPipeline);
        limelight.start();

        drive.init(hardwareMap);
        shooter.init(hardwareMap);

        timer.reset();

        if (limelight.isConnected() && limelight.isRunning()) {;
            telemetry.addLine("Limelight Connected");
            telemetry.addLine("Ready to start");
        } else {
            telemetry.addLine("Limelight is not Connected");
            telemetry.addLine("Please reconnect the Limelight cable");
        }
    }

    @Override
    public void loop() {

        if (gamepad1.bWasPressed()) {
            resetPose();
        }
        follower.update();

        // Automatic Flywheel and Hood
        if (gamepad2.xWasPressed()) {
            targetVelocity = 0;
            autoShooterEnabled = !autoShooterEnabled;
        }
        if (gamepad2.yWasPressed()) {
            autoHoodEnabled = !autoHoodEnabled;
        }
        updateDistance();
        autoFlywheel();
        autoHood();

        // Auto Park
        if(gamepad1.y && !autoDriving) {
            goToParkPose();
            autoDriving = true;
        }
        if(autoDriving && driverOverride()) {
            follower.breakFollowing();
            drive.stopDrive();
            drive.setBrake();
            autoDriving = false;
        }

        // Drive Control
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
            Aimbot(limelight.getLatestResult());
            lockHeading();
            drive.drive(forward,strafe,rotate);
        }

        // Subsystem
        subSystem();
        shooter.update();
        indicateFullBall();

        // Telemetry
        telemetry.addLine("-------------- Shooter ------------");
        telemetry.addData("Target Vel:", targetVelocity);
        telemetry.addData("Current Vel:", shooter.getFlywheelVel1());
        telemetry.addData("Hood", shooter.getHoodPos());
        telemetry.addData("Gate", shooter.getGatePos());
        telemetry.addData("Shooter Power:", shooter.getFlywheelPower1());
        telemetry.addData("Intake Power:", shooter.getIntakePower());
        telemetry.addData("IntakeOn", intakeOn);
        telemetry.addData("AutoFlywheelOn", autoShooterEnabled);
        telemetry.addData("AutoHoodOn", autoHoodEnabled);
        telemetry.addLine("");
        telemetry.addLine("-------------- Offset ------------");
        telemetry.addData("Flywheel Offset", FlywheelManualOffset);
        telemetry.addData("Hood Offset", HoodOffset);
        telemetry.addLine("");
        telemetry.addLine("-------------- Pose ------------");
        telemetry.addData("Distance", drive.getDistanceToGoal(follower.getPose(),GOAL));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine("");
        telemetry.addLine("-------------- Camera ------------");
        telemetry.addData("CameraTagDetected?", limelight.getLatestResult().isValid());
        telemetry.addData("PipelineIndex", limelight.getStatus().getPipelineIndex());
        telemetry.addData("Connected", limelight.isConnected());
        telemetry.addData("Running", limelight.isRunning());
        telemetry.addLine("");
        telemetry.addLine("-------------- Gamepad1 ------------");
        telemetry.addLine("left/right stick - MecanumDrive");
        telemetry.addLine("Dpad U/D - AdjustFlywheelVel");
        telemetry.addLine("A - ToggleIntake");
        telemetry.addLine("B - ResetPose");
        telemetry.addLine("X - ResetIntake");
        telemetry.addLine("Y - AutoPark");
        telemetry.addLine("RB - Shot");
        telemetry.addLine("RT - TurnTo180degrees");
        telemetry.addLine("LB - LimelightAim");
        telemetry.addLine("LT - OdometryAim");
        telemetry.addLine("");
        telemetry.addLine("-------------- Gamepad2 ------------");
        telemetry.addLine("Dpad U/D - AdjustHood & HoodOffset");
        telemetry.addLine("Dpad L/R - AdjustFlywheel & FlywheelOffset");
        telemetry.addLine("X - AutoFlywheel");
        telemetry.addLine("Y - AutoHood");
        telemetry.addLine("");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.shutdown();
    }

    public void goToParkPose() {

        PathChain shootPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        PoseConstant.BluePark
                ))
                .setConstantHeadingInterpolation(follower.getPose().getHeading())
                .build();

        follower.followPath(shootPath);
    }

    public void manualDrive() {
        strafe = gamepad1.left_stick_x;
        forward = -gamepad1.left_stick_y;
        rotate = gamepad1.right_stick_x;
    }

    private double rampTowardsTarget(double current, double target, double rate) {
        double delta = target - current;
        if (Math.abs(delta) > rate) {
            return current + Math.signum(delta) * rate;
        } else {
            return target;
        }
    }

    public boolean driverOverride() {
        return Math.abs(gamepad1.left_stick_x) > 0.15 ||
                Math.abs(gamepad1.left_stick_y) > 0.15 ||
                Math.abs(gamepad1.right_stick_x) > 0.15;
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
        if (gamepad1.x) {
            shooter.setReturnRequested();
        }
//        shooter.setIntakeBoost(gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5);

        // Shooter
        if (gamepad1.right_bumper) {
            shooter.fireManualOn();
        } else {
            shooter.fireManualOff();
        }


        // Hood
        shooter.setHood(HoodPos);

    }

    public void Aimbot(LLResult result) {
        if (gamepad1.left_bumper && gamepad1.left_trigger > 0.5) {
            double dt = timer.seconds();
            timer.reset();
            if (result.isValid()) {
                // ===== CAMERA AIM =====
                double error = result.getTx() + ShooterConstant.llOffset;
                double pTerm = error * ShooterConstant.llkP;
                double dTerm = 0;
                if (dt > 0) {
                    dTerm = ((error - lastErrorLL) / dt) * ShooterConstant.llkD;
                }
                double rawRotate = Range.clip(pTerm + dTerm, -ShooterConstant.maxRotatePower, ShooterConstant.maxRotatePower);
                if (Math.abs(rawRotate) < ShooterConstant.lowPowerThreshold) {
                    rotate = Math.signum(rawRotate) * Math.pow(Math.abs(rawRotate), ShooterConstant.exponent);
                } else {
                    rotate = rawRotate;
                }
                lastErrorLL = error;
                telemetry.addLine("Camera in use");
                telemetry.addData("error", error);
                telemetry.addLine("");
            } else {
                // ===== ODOMETRY AIM =====
                Pose robotPose = follower.getPose();
                double targetHeading = drive.getAngleToGoal(robotPose, GOAL);
                double error = Util.angleWrap(robotPose.getHeading() - targetHeading + ShooterConstant.odoOffset);
                double pTerm = error * ShooterConstant.odokP;
                double dTerm = 0;
                if (dt > 0) {
                    dTerm = ((error - lastErrorOdo) / dt) * ShooterConstant.odokD;
                }
                double rawRotate = Range.clip(pTerm + dTerm, -ShooterConstant.maxRotatePower, ShooterConstant.maxRotatePower);
                if (Math.abs(rawRotate) < ShooterConstant.lowPowerThreshold) {
                    rotate = Math.signum(rawRotate) * Math.pow(Math.abs(rawRotate), ShooterConstant.exponent);
                } else {
                    rotate = rawRotate;
                }
                lastErrorOdo = error;
                telemetry.addLine("Odometry in use");
                telemetry.addData("error", error);
                telemetry.addLine("");
            }
        } else if (gamepad1.left_trigger > 0.5) {
            // ===== ODOMETRY AIM =====
            double dt = timer.seconds();
            timer.reset();
            Pose robotPose = follower.getPose();
            double targetHeading = drive.getAngleToGoal(robotPose, GOAL);
            double error = Util.angleWrap(robotPose.getHeading() - targetHeading + ShooterConstant.odoOffset);
            double pTerm = error * ShooterConstant.odokP;
            double dTerm = 0;
            if (dt > 0) {
                dTerm = ((error - lastErrorOdo) / dt) * ShooterConstant.odokD;
            }
            double rawRotate = Range.clip(pTerm + dTerm, -ShooterConstant.maxRotatePower, ShooterConstant.maxRotatePower);
            if (Math.abs(rawRotate) < ShooterConstant.lowPowerThreshold) {
                rotate = Math.signum(rawRotate) * Math.pow(Math.abs(rawRotate), ShooterConstant.exponent);
            } else {
                rotate = rawRotate;
            }
            lastErrorOdo = error;
            telemetry.addLine("Odometry in use");
            telemetry.addData("error", error);
            telemetry.addLine("");
        } else if (gamepad1.left_bumper) {
            if (result.isValid()) {
                // ===== CAMERA AIM =====
                double dt = timer.seconds();
                timer.reset();
                double error = result.getTx() + ShooterConstant.llOffset;
                double pTerm = error * ShooterConstant.llkP;
                double dTerm = 0;
                if (dt > 0) {
                    dTerm = ((error - lastErrorLL) / dt) * ShooterConstant.llkD;
                }
                double rawRotate = Range.clip(pTerm + dTerm, -ShooterConstant.maxRotatePower, ShooterConstant.maxRotatePower);
                if (Math.abs(rawRotate) < ShooterConstant.lowPowerThreshold) {
                    rotate = Math.signum(rawRotate) * Math.pow(Math.abs(rawRotate), ShooterConstant.exponent);
                } else {
                    rotate = rawRotate;
                }
                lastErrorLL = error;
                telemetry.addLine("Camera in use");
                telemetry.addData("error", error);
                telemetry.addLine("");
            }
        }
    }

    public void updateDistance() {
        Pose robotPose = follower.getPose();
        double rawDistance = drive.getDistanceToGoal(robotPose, GOAL);
        filteredDistance = 0 * filteredDistance + 1 * rawDistance;
    }

    public void autoFlywheel() {

        if (!autoShooterEnabled) {

//            if (gamepad1.yWasPressed()) {
//                targetVelocity = FarVel;
//            }
//            if (gamepad1.bWasPressed()) {
//                targetVelocity = ZeroVel;
//            }
            if (gamepad1.dpadUpWasPressed() || gamepad2.dpadRightWasPressed()) {
                targetVelocity += VelStep;
            }
            if (gamepad1.dpadDownWasPressed() || gamepad2.dpadLeftWasPressed()) {
                targetVelocity -= VelStep;
            }

            targetVelocity = Math.max(0, targetVelocity);

        } else {

            setFlywheelManualOffset();

            double distance = filteredDistance;
            distance = Range.clip(distance,0,170);
            targetVelocity = Util.getFlywheelVelocityFromDistance(distance) + FlywheelManualOffset;

        }
    }

    public void autoHood() {

        if (!autoHoodEnabled) {

            if (gamepad2.dpadUpWasPressed()) {
                HoodPos += ShooterConstant.hoodStep;
            }
            if (gamepad2.dpadDownWasPressed()) {
                HoodPos -= ShooterConstant.hoodStep;
            }
            HoodPos = Range.clip(HoodPos, ShooterConstant.minHoodPos, ShooterConstant.maxHoodPos);

        } else {

            setHoodOffset();

            double distance = filteredDistance;
            distance = Range.clip(distance,0,170);
            HoodPos = Util.getHoodPositionFromDistance(distance) + HoodOffset;

        }
    }

    public void setFlywheelManualOffset() {
        if (gamepad2.dpadRightWasReleased()) {
            FlywheelManualOffset += 20;
        } else if (gamepad2.dpadLeftWasPressed()) {
            FlywheelManualOffset -= 20;
        }
    }

    public void setHoodOffset() {
        if (gamepad2.dpadUpWasPressed()) {
            HoodOffset += 0.01;
        } else if (gamepad2.dpadDownWasPressed()) {
            HoodOffset -= 0.01;
        }
    }

    public void indicateFullBall() {
        boolean currentFull = shooter.isThreeBall();
        if (currentFull && !lastFull) {
            gamepad1.rumble(250);
        }
        lastFull = currentFull;
    }

    public void lockHeading() {
        if (gamepad1.right_trigger > 0.5) {
            double targetHeading = Math.toRadians(180);
            double currentHeading = follower.getPose().getHeading();
            double error = Util.angleWrap(currentHeading - targetHeading);
            double rotateAssist = error * 0.5;
            rotateAssist = Range.clip(rotateAssist, -ShooterConstant.maxRotatePower, ShooterConstant.maxRotatePower);
            rotate = rotateAssist;
        }
    }

    public void resetPose() {
        follower.setPose(PoseConstant.BlueResetPose);
    }

}