package org.firstinspires.ftc.teamcode.Autonomous.Red.Main;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.mechanism.ShooterV6;
import org.firstinspires.ftc.teamcode.mechanism.Util;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red-Far-Gamma",group = "Red")
@Configurable
public class RedFarGamma extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    // ------ Mechanics Setup ------- //
    private ShooterV6 shooter = new ShooterV6();
    private boolean shotsTriggered = false;
    private double targetVelocity = 0;
    private double filteredDistance = 36;
    private double HoodPos = 0;
    private double targetHeading = 0;

    // Pose
    private final Pose startPose = PoseConstant.RedFarAutoStartPose;
    private static final Pose GOAL = PoseConstant.RED_GOAL;
    private final ElapsedTime timer = new ElapsedTime();
    private enum PathState {
        toShootPreload,
        toCollect3,
        toShootC3,
        toCollectHuman1,
        toShootHuman1,
        toBall1,
        toShootBall1,
        toBall2,
        toShootBall2,
        toBall3,
        toShootBall3,
        toBall4,
        toShootBall4,
        Finished,
        toPark,
        Aiming,
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

        shooter.init(hardwareMap);

        timer.reset();

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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

        targetVelocity = Range.clip(targetVelocity,0,1520);
        shooter.flywheelOn(targetVelocity);
        shooter.setHood(HoodPos);

        autonomousPathUpdate();

        PoseConstant.AutoEndPose = follower.getPose();
        PoseConstant.hasAutoPose = true;

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
                shooter.setIntakeBoost(true);
                shooter.outtakeOn();
                shooter.fullBall();
                follower.followPath(toShootPreload, true);
                setPathState(PathState.toCollect3);
                break;

            case toCollect3:
                if (!follower.isBusy() && (shooter.getFlywheelVel1() > targetVelocity - 100 || pathTimer.getElapsedTimeSeconds() > 2.267)) {
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
                    shooter.fullBall();
                    follower.followPath(toShootC3, true);
                    setPathState(PathState.toCollectHuman1);
                }
                break;

            case toCollectHuman1:
                if (!follower.isBusy() && (shooter.getFlywheelVel1() > targetVelocity - 100 || pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toCollectHuman1);
                        setPathState(PathState.toShootHuman1);
                    }
                }
                break;

            case toShootHuman1:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    shooter.fullBall();
                    follower.followPath(toShootHuman1, true);
                    setPathState(PathState.toBall1);
                }
                break;

            case toBall1:
                if (!follower.isBusy() && (shooter.getFlywheelVel1() > targetVelocity - 100 || pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toBall1);
                        setPathState(PathState.toShootBall1);
                    }
                }
                break;

            case toShootBall1:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2) {
                    shooter.fullBall();
                    follower.followPath(toShootBall1, true);
                    setPathState(PathState.toBall2);
                }
                break;

            case toBall2:
                if (!follower.isBusy() && (shooter.getFlywheelVel1() > targetVelocity - 100 || pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toBall2);
                        setPathState(PathState.toShootBall2);
                    }
                }
                break;

            case toShootBall2:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2) {
                    shooter.fullBall();
                    follower.followPath(toShootBall2, true);
                    setPathState(PathState.toBall3);
                }
                break;

            case toBall3:
                if (!follower.isBusy() && (shooter.getFlywheelVel1() > targetVelocity - 100 || pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toBall3);
                        setPathState(PathState.toShootBall3);
                    }
                }
                break;

            case toShootBall3:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2) {
                    shooter.fullBall();
                    follower.followPath(toShootBall3, true);
                    setPathState(PathState.toBall4);

                }
                break;

            case toBall4:
                if (!follower.isBusy() && (shooter.getFlywheelVel1() > targetVelocity - 100 || pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toBall4);
                        setPathState(PathState.toShootBall4);
                    }
                }
                break;

            case toShootBall4:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2) {
                    shooter.fullBall();
                    follower.followPath(toShootBall4, true);
                    setPathState(PathState.Finished);

                }
                break;

            case Finished:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        shooter.fullBall();
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
    public PathChain toCollect3;
    public PathChain toShootC3;
    public PathChain toCollectHuman1;
    public PathChain toShootHuman1;
    public PathChain toBall1;
    public PathChain toShootBall1;
    public PathChain toBall2;
    public PathChain toShootBall2;
    public PathChain toBall3;
    public PathChain toShootBall3;
    public PathChain toBall4;
    public PathChain toShootBall4;
    public PathChain toPark;
    public void buildPaths() {

        toShootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(96.000, 9.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                .build();

        toCollect3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.000, 19.000),
                                new Pose(86.967, 36.850),
                                new Pose(133.000, 35.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootC3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 35.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                .build();

        toCollectHuman1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(135.000, 8.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootHuman1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 8.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                .build();

        toBall1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(135.000, 8.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootBall1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 8.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();

        toBall2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(135.000, 8.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootBall2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 8.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                .build();

        toBall3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(135.000, 8.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootBall3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 8.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                .build();

        toBall4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(135.000, 24.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootBall4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 24.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();

        toPark = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(89.748, 29.841)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

    private void updateDistance() {
        Pose robotPose = follower.getPose();
        double rawDistance = Util.getDistanceToGoal(robotPose, GOAL);
        filteredDistance = 0 * filteredDistance + 1 * rawDistance;
    }

    private void autoFlywheel() {
        double distance = filteredDistance;
        targetVelocity = Util.getFlywheelVelocityFromDistance(distance)-10;
    }

    private void autoHood() {
        double distance = filteredDistance;
        HoodPos = Util.getHoodPositionFromDistance(distance);
    }

}