package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.ShooterV6;
import org.firstinspires.ftc.teamcode.mechanism.ShooterV6Far;
import org.firstinspires.ftc.teamcode.mechanism.Util;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Red-V6-Far",group = "Red")
@Configurable
public class RedV6Far extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    // ------ Mechanics Setup ------- //
    private MecanumDrive drive = new MecanumDrive();
    private ShooterV6Far shooter = new ShooterV6Far();
    private boolean shotsTriggered = false;
    private double targetVelocity = 0;
    private double filteredDistance = 36;
    private double HoodPos = 0;

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
        toCollectHuman2,
        toShootHuman2,
        toCollect32,
        toShootC32,
        toCollectHuman3,
        toShootHuman3,
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
                shooter.fullBall();
                follower.followPath(toShootPreload, true);
                setPathState(PathState.toCollectHuman1);
                break;

            case toCollectHuman1:
                if (!follower.isBusy() && shooter.getFlywheelVel1() > targetVelocity - 100) {
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
                if (!follower.isBusy()) {
                    follower.followPath(toShootHuman1, true);
                    setPathState(PathState.toCollect3);
                }
                break;

            case toCollect3:
                if (!follower.isBusy() && shooter.getFlywheelVel1() > targetVelocity - 100) {
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
                    setPathState(PathState.toCollectHuman2);
                }
                break;

            case toCollectHuman2:
                if (!follower.isBusy() && shooter.getFlywheelVel1() > targetVelocity - 100) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toCollectHuman2);
                        setPathState(PathState.toShootHuman2);
                    }
                }
                break;

            case toShootHuman2:
                if (!follower.isBusy()) {
                    follower.followPath(toShootHuman2, true);
                    setPathState(PathState.toCollect32);
                }
                break;

            case toCollect32:
                if (!follower.isBusy() && shooter.getFlywheelVel1() > targetVelocity - 100) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toCollect32);
                        setPathState(PathState.toShootC32);
                    }
                }
                break;

            case toShootC32:
                if (!follower.isBusy()) {
                    follower.followPath(toShootC32, true);
                    setPathState(PathState.toCollectHuman3);
                }
                break;

            case toCollectHuman3:
                if (!follower.isBusy() && shooter.getFlywheelVel1() > targetVelocity - 100) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toCollectHuman3);
                        setPathState(PathState.toShootHuman3);
                    }
                }
                break;

            case toShootHuman3:
                if (!follower.isBusy()) {
                    follower.followPath(toShootHuman3, true);
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
    public PathChain toCollect3;
    public PathChain toShootC3;
    public PathChain toCollectHuman2;
    public PathChain toShootHuman2;
    public PathChain toCollectHuman1;
    public PathChain toShootHuman1;
    public PathChain toCollect32;
    public PathChain toShootC32;
    public PathChain toCollectHuman3;
    public PathChain toShootHuman3;
    public PathChain toPark;
    public void buildPaths() {

        toShootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(96.000, 9.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
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
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                .build();

        toCollectHuman2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(133.000, 8.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootHuman2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 8.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                .build();

        toCollect32 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(133.000, 35.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        toShootC32 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 35.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(19), Math.toRadians(67))
                .build();

        toCollectHuman3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(133.000, 8.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootHuman3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 8.000),
                                new Pose(86.000, 19.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                .build();

        toPark = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 19.000),
                                new Pose(92.121, 33.383)
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