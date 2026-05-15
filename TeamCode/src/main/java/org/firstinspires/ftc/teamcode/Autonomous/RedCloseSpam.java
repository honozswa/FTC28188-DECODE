package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.TurretConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.ShooterV2;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.mechanism.Util;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "RedSpam")
@Configurable
public class RedCloseSpam extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    // ------ Flywheel Setup ------- //
    private MecanumDrive drive = new MecanumDrive();
    private ShooterV2 shooter = new ShooterV2();
    private Turret turret = new Turret();
    private Limelight3A limelight;
    private boolean shotsTriggered = false;
    private double startVel = 1260;
    private double rampVel = 1400;
    // Pose
    private final Pose startPose = PoseConstant.RedCloseAutoStartPose;
    private static final Pose GOAL = PoseConstant.RED_GOAL;

    // Camera Variables
    double lastErrorCam = 0;
    double lastErrorOdo = 0;

    // Turret
    boolean turretOn = true;
    double turretPower = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private enum PathState {
        toShootPreload,
        toShootC1,
        toShootC2,
        toShootC3,
        toShootR1,
        toShootR2,
        toShootR3,
        toShootR4,
        toCollect1,
        toCollect2,
        toCollect3,
        toRampCollect1,
        toRampCollect2,
        toRampCollect3,
        toRampCollect4,
        Finished,
        toPark,
        Idle
    }
    PathState pathState;

    @Override
    public void init() {
        pathState = PathState.toShootPreload;
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // init other mech
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4); // BlueTag
        limelight.start();

        turret.init(hardwareMap);
        shooter.init(hardwareMap);
        shooter.setHood(0);

        timer.reset();

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

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
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {

        follower.update();
        shooter.update();
        TurretAim(limelight.getLatestResult());
        autonomousPathUpdate();

//        PoseConstant.AutoEndPose = follower.getPose();
//        TurretConstant.TurretAngleOffset = turret.getCurrentAngle();

        panelsTelemetry.debug("Path State", pathState.toString());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        PoseConstant.hasAutoPose = true;
        TurretConstant.hasTurretAngle = true;
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case toShootPreload:
                shooter.flywheelOn(startVel);
                shooter.intakeOn();
                shooter.fullBall();
                follower.followPath(toShootPreload, true);
                setPathState(PathState.toCollect2);
                break;

            case toCollect2:
                if (!follower.isBusy()) {
                    if (!shotsTriggered && shooter.getFlywheelVel1() > startVel - 120) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toCollect2);
                        setPathState(PathState.toShootC2);
                    }
                }
                break;

            case toShootC2:
                if (!follower.isBusy()) {
                    follower.followPath(toShootC2, true);
                    shooter.flywheelOn(rampVel);
                    shooter.setHood(0.3);
                    setPathState(PathState.toRampCollect1);
                }
                break;

            case toRampCollect1:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toRampCollect1);
                        setPathState(PathState.toShootR1);
                    }
                }
                break;

            case toShootR1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(toShootR1, true);
                    setPathState(PathState.toRampCollect2);
                }
                break;

            case toRampCollect2:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toRampCollect2);
                        setPathState(PathState.toShootR2);
                    }
                }
                break;

            case toShootR2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(toShootR2, true);
                    setPathState(PathState.toRampCollect4);
                }
                break;

//            case toRampCollect3:
//                if (!follower.isBusy()) {
//                    if (!shotsTriggered) {
//                        shooter.fireShot();
//                        shotsTriggered = true;
//                    } else if (!shooter.isBusy()) {
//                        follower.followPath(toRampCollect3);
//                        setPathState(PathState.toShootR3);
//                    }
//                }
//                break;
//
//            case toShootR3:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
//                    follower.followPath(toShootR3, true);
//                    setPathState(PathState.toRampCollect4);
//                }
//                break;

            case toRampCollect4:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toRampCollect4);
                        setPathState(PathState.toShootR4);
                    }
                }
                break;

            case toShootR4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(toShootR4, true);
                    setPathState(PathState.toCollect1);
                }
                break;

            case toCollect1:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toCollect1);
                        setPathState(PathState.toShootC1);
                    }
                }
                break;

            case toShootC1:
                if (!follower.isBusy()) {
                    follower.followPath(toShootC1, true);
                    setPathState(PathState.Finished);

                }
                break;

//            case toCollect3:
//                if (!follower.isBusy()) {
//                    if (!shotsTriggered) {
//                        shooter.fireShot();
//                        shotsTriggered = true;
//                    } else if (!shooter.isBusy()) {
//                        follower.followPath(toCollect3);
//                        setPathState(PathState.toShootC3);
//                    }
//                }
//                break;
//
//            case toShootC3:
//                if (!follower.isBusy()) {
//                    follower.followPath(toShootC3, true);
//                    setPathState(PathState.Finished);
//                }
//                break;

            case Finished:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        setPathState(PathState.toPark);
                    }
                }
                break;

            case toPark:
                if (!follower.isBusy()) {
                    follower.followPath(toPark, true);
                    setPathState(PathState.Idle);
                }
                break;
        }
    }

    // PathChain
    public PathChain toShootPreload;
    public PathChain toCollect2;
    public PathChain toShootC2;
    public PathChain toRampCollect1;
    public PathChain toShootR1;
    public PathChain toRampCollect2;
    public PathChain toShootR2;
    public PathChain toRampCollect3;
    public PathChain toShootR3;
    public PathChain toRampCollect4;
    public PathChain toShootR4;
    public PathChain toCollect1;
    public PathChain toShootC1;
    public PathChain toCollect3;
    public PathChain toShootC3;
    public PathChain toPark;
    public void buildPaths() {

        toShootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(96.000, 135.000),
                                new Pose(96.000, 95.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        toCollect2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.000, 95.000),
                                new Pose(93.000, 63.033),
                                new Pose(133.000, 55.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        toShootC2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 55.000),
                                new Pose(88.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();

        toRampCollect1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.000, 75.000),
                                new Pose(91.000, 63.000),
                                new Pose(132.000, 62.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(30))
                .build();

        toShootR1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.000, 62.000),
                                new Pose(88.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(270))
                .setNoDeceleration()
                .build();

        toRampCollect2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.000, 75.000),
                                new Pose(132.000, 62.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(30))
                .build();

        toShootR2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.000, 62.000),
                                new Pose(88.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(270))
                .setNoDeceleration()
                .build();

        toRampCollect3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.000, 75.000),
                                new Pose(132.000, 62.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(30))
                .build();

        toShootR3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.000, 62.000),
                                new Pose(88.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(270))
                .setNoDeceleration()
                .build();

        toRampCollect4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.000, 75.000),
                                new Pose(132.000, 62.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(30))
                .build();

        toShootR4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.000, 62.000),
                                new Pose(88.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(270))
                .setNoDeceleration()
                .build();

        toCollect1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.000, 75.000),
                                new Pose(88.192, 84.921),
                                new Pose(129.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootC1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(129.000, 84.000),
                                new Pose(96.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();

        toCollect3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.000, 84.000),
                                new Pose(99.000, 27.000),
                                new Pose(91.000, 37.000),
                                new Pose(135.000, 35.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootC3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 35.000),
                                new Pose(96.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();

        toPark = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(96.000, 84.000),
                                new Pose(98.000, 68.571)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

    public void TurretAim(LLResult result) {
        if (turretOn) {
            Pose robotPose = follower.getPose();
            double dt = timer.seconds();
            timer.reset();
            if (result.isValid() && drive.getDistanceToGoal(robotPose, GOAL) > 140) {
                // ===== CAMERA AIM =====
                double turretAngle = Math.toRadians(turret.getCurrentAngle());
                double txRad = Math.toRadians(result.getTx());
                double turretTarget = Util.angleWrap(turretAngle - txRad);
                double clippedTarget = Range.clip(turretTarget, TurretConstant.MIN_ANGLE, TurretConstant.MAX_ANGLE);
                double error = clippedTarget - turretAngle;
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

                telemetry.addLine("Camera in use");
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

                turret.setPower(Range.clip(turretPower,-TurretConstant.MAX_POWER,TurretConstant.MAX_POWER));

                telemetry.addLine("Odometry in use");
                telemetry.addLine("");

            }
        }
    }

}