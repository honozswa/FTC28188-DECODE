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

@TeleOp(name = "Red-V6-OpMode", group = "Main")
public class RedV6OpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Follower follower;
    ShooterV6 shooter = new ShooterV6();
    private Limelight3A limelight;

    // Drivetrain
    double forward = 0, strafe = 0, rotate = 0;

    // Flywheel
    double targetVelocity = 0;
    boolean autoShooterEnabled = true;
    double ZeroVel = ShooterConstant.ZeroVel;
    double FarVel = ShooterConstant.FarVel;
    final double VelStep = ShooterConstant.VelStep;

    // Intake
    private boolean intakeOn = false;

    // Hood
    double HoodPos = ShooterConstant.minHoodPos;
    boolean autoHoodEnabled = true;

    // Pose
    private static final Pose GOAL = PoseConstant.RED_GOAL;
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
        follower.setStartingPose(PoseConstant.RedCloseAutoStartPose);
        }

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(ShooterConstant.redTagPipeline);
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
        if(gamepad1.left_bumper && !autoDriving) {
            goToShootPose();
            autoDriving = true;
        }
        if(autoDriving && drive.driverOverride()) {
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
            drive.drive(forward,strafe,rotate);
        }

        // Subsystem
        subSystem();
        shooter.update();

        // Telemetry
        telemetry.addLine("-------------- Shooter ------------");
        telemetry.addData("Target Vel:", targetVelocity);
        telemetry.addData("Current Vel:", shooter.getFlywheelVel1());
        telemetry.addData("Hood", shooter.getHoodPos());
        telemetry.addData("Gate", shooter.getGatePos());
        telemetry.addData("Shooter Power:", shooter.getFlywheelPower1());
        telemetry.addData("Intake Power:", shooter.getIntakePower());
        telemetry.addData("AutoFlywheelOn", autoShooterEnabled);
        telemetry.addData("AutoHoodOn", autoHoodEnabled);
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
        telemetry.addData("Connected", limelight.isConnected());
        telemetry.addData("Running", limelight.isRunning());
        telemetry.addLine("");
        telemetry.addLine("-------------- Gamepad1 ------------");
        telemetry.addLine("MecanumDrive: left/right stick");
        telemetry.addLine("ToggleIntake: A");
        telemetry.addLine("IntakeSpeedUp: RT");
        telemetry.addLine("Shot: RB");
        telemetry.addLine("FlywheelFarVel: Y");
        telemetry.addLine("FlywheelOff: B");
        telemetry.addLine("AdjustFlywheelVel: Dpad Up/Down");
        telemetry.addLine("AutoPark: LB");
        telemetry.addLine("ResetIntake: LT");
        telemetry.addLine("");
        telemetry.addLine("-------------- Gamepad2 ------------");
        telemetry.addLine("AutoFlywheel: X");
        telemetry.addLine("AutoHood: Y");
        telemetry.addLine("AutoTurret: A");
        telemetry.addLine("ResetTurretAngle: LB");
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

        // Shooter
        if (gamepad1.rightBumperWasPressed()) {
            shooter.fireShot();
        }
        if (gamepad1.x) {
            shooter.setReturnRequested();
        }

        // Hood
        shooter.setHood(HoodPos);

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
                telemetry.addLine("");
            }
        }
    }

    public void updateDistance() {
        Pose robotPose = follower.getPose();
        double rawDistance = drive.getDistanceToGoal(robotPose, GOAL);
        filteredDistance = 0.8 * filteredDistance + 0.2 * rawDistance;
    }

    public void autoFlywheel() {

        if (!autoShooterEnabled) {

            if (gamepad1.yWasPressed()) {
                targetVelocity = FarVel;
            }
            if (gamepad1.bWasPressed()) {
                targetVelocity = ZeroVel;
            }
            if (gamepad1.dpadUpWasPressed()) {
                targetVelocity += VelStep;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetVelocity -= VelStep;
            }

            targetVelocity = Math.max(0, targetVelocity);

        } else {

            double distance = filteredDistance;
            targetVelocity = Util.getFlywheelVelocityFromDistance(distance);

        }
    }

    public void autoHood() {

        if (!autoHoodEnabled) {

            if (gamepad2.dpadUpWasPressed()) {
                HoodPos += 0.05;
            }
            if (gamepad2.dpadDownWasPressed()) {
                HoodPos -= 0.05;
            }
            HoodPos = Range.clip(HoodPos, ShooterConstant.minHoodPos, ShooterConstant.maxHoodPos);

        } else {

            double distance = filteredDistance;
            HoodPos = Util.getHoodPositionFromDistance(distance);

        }
    }

}