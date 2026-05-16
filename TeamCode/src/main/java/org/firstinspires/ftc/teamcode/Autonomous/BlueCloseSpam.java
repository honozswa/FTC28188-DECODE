package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.ShooterV6;
import org.firstinspires.ftc.teamcode.mechanism.Util;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueSpam")
@Configurable
public class BlueCloseSpam extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    // ------ Mechanics Setup ------- //
    private MecanumDrive drive = new MecanumDrive();
    private ShooterV6 shooter = new ShooterV6();
    private Limelight3A limelight;
    private boolean shotsTriggered = false;
    private double targetVelocity = 0;
    private double filteredDistance = 36;
    private double HoodPos = 0;

    // Pose
    private final Pose startPose = PoseConstant.BlueCloseAutoStartPose;
    private static final Pose GOAL = PoseConstant.BLUE_GOAL;
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
        limelight.pipelineSwitch(ShooterConstant.blueTagPipeline);
        limelight.start();

        shooter.init(hardwareMap);

        timer.reset();

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        if (limelight.isConnected() && limelight.isRunning()) {;
            telemetry.addLine("Limelight Connected");
            telemetry.addLine("Ready to start");
        } else {
            telemetry.addLine("Limelight is not Connected");
            telemetry.addLine("Please reconnect the Limelight cable");
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

        updateDistance();
        autoFlywheel();
        autoHood();

        autonomousPathUpdate();

        PoseConstant.AutoEndPose = follower.getPose();

        panelsTelemetry.debug("Path State", pathState.toString());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        PoseConstant.hasAutoPose = true;
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case toShootPreload:
                shooter.intakeOn();
                shooter.fullBall();
                follower.followPath(toShootPreload, true);
                setPathState(PathState.toCollect2);
                break;

            case toCollect2:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
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
//                    setPathState(PathState.toCollect1);
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
                    setPathState(PathState.toCollect3);

                }
                break;

            case toCollect3:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toCollect3);
                        setPathState(PathState.toShootC3);
                    }
                }
                break;

            case toShootC3:
                if (!follower.isBusy()) {
                    follower.followPath(toShootC3, true);
                    setPathState(PathState.Finished);
                }
                break;

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
                                new Pose(48.000, 135.000),
                                new Pose(48.000, 80.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        toCollect2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.000, 80.000),
                                new Pose(51.000, 48.000),
                                new Pose(27.692, 57.290),
                                new Pose(11.000, 55.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toShootC2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.000, 55.000),
                                new Pose(56.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setNoDeceleration()
                .build();

        toRampCollect1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 75.000),
                                new Pose(53.000, 63.000),
                                new Pose(10.000, 58.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        toShootR1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 58.000),
                                new Pose(56.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .setNoDeceleration()
                .build();

        toRampCollect2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                new Pose(10.000, 58.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        toShootR2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 58.000),
                                new Pose(56.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .setNoDeceleration()
                .build();

        toRampCollect3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                new Pose(10.000, 58.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        toShootR3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 58.000),
                                new Pose(56.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .setNoDeceleration()
                .build();

        toRampCollect4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                new Pose(10.000, 58.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        toShootR4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 58.000),
                                new Pose(56.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        toCollect1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                new Pose(15.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toShootC1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.000, 84.000),
                                new Pose(48.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setNoDeceleration()
                .build();

        toCollect3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.000, 84.000),
                                new Pose(45.000, 27.000),
                                new Pose(53.000, 37.000),
                                new Pose(9.000, 35.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toShootC3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.000, 35.000),
                                new Pose(56.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setNoDeceleration()
                .build();

        toPark = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 75.000),
                                new Pose(46.741, 68.571)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

    private void updateDistance() {
        Pose robotPose = follower.getPose();
        double rawDistance = drive.getDistanceToGoal(robotPose, GOAL);
        filteredDistance = 0.8 * filteredDistance + 0.2 * rawDistance;
    }

    private void autoFlywheel() {
        double distance = filteredDistance;
        targetVelocity = Util.getFlywheelVelocityFromDistance(distance);
    }

    private void autoHood() {
        double distance = filteredDistance;
        HoodPos = Util.getHoodPositionFromDistance(distance);
    }

}