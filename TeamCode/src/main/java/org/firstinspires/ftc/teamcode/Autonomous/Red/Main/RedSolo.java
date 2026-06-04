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
import org.firstinspires.ftc.teamcode.mechanism.ShooterV6;
import org.firstinspires.ftc.teamcode.mechanism.Util;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Autonomous(name = "Red-Close-Solo",group = "Red")
@Configurable
public class RedSolo extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    // ------ Mechanics Setup ------- //
    private ShooterV6 shooter = new ShooterV6();
    private boolean shotsTriggered = false;
    private double targetVelocity = 0;
    private double filteredDistance = 36;
    private double HoodPos = 0;

    // Pose
    private final Pose startPose = PoseConstant.RedCloseAutoStartPose;
    private static final Pose GOAL = PoseConstant.RED_GOAL;
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
        Idle,
        toOpenGate1
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
                setPathState(PathState.toCollect2);
                break;

            case toCollect2:
                if (!follower.isBusy() && shooter.getFlywheelVel1() > targetVelocity - 100) {
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
                    shooter.fullBall();
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
                if (!follower.isBusy() && (pathTimer.getElapsedTimeSeconds() > 2.9 || shooter.isThreeBall())) {
                    shooter.fullBall();
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
                if (!follower.isBusy() && (pathTimer.getElapsedTimeSeconds() > 3 || shooter.isThreeBall())) {
                    shooter.fullBall();
                    follower.followPath(toShootR2, true);
                    setPathState(PathState.toRampCollect3);
                }
                break;

            case toRampCollect3:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(toRampCollect3);
                        setPathState(PathState.toShootR3);
                    }
                }
                break;

            case toShootR3:
                if (!follower.isBusy() && (pathTimer.getElapsedTimeSeconds() > 3 || shooter.isThreeBall())) {
                    shooter.fullBall();
                    follower.followPath(toShootR3, true);
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
                    shooter.fullBall();
                    follower.followPath(toShootC1, true);
                    setPathState(PathState.Finished);
                }
                break;

            case Finished:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        setPathState(PathState.Idle);
                    }
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
    public PathChain toCollect1;
    public PathChain toShootC1;
    public void buildPaths() {

        toShootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(96.000, 135.000),
                                new Pose(96.000, 95.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(40))
                .build();

        toCollect2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.000, 95.000),
                                new Pose(95.000, 53.832),
                                new Pose(136.000, 55.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootC2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(136.000, 55.000),
                                new Pose(85.000, 80.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                .build();

        toRampCollect1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 80.000),
                                new Pose(133.000, 59.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(28))
                .build();

        toShootR1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 59.000),
                                new Pose(85.000, 80.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(50.5))
                .build();

        toRampCollect2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 80.000),
                                new Pose(133.000, 59.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50.5), Math.toRadians(28))
                .build();

        toShootR2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 59.000),
                                new Pose(85.000, 80.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(50.5))
                .build();

        toRampCollect3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 80.000),
                                new Pose(133.000, 59.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50.5), Math.toRadians(28))
                .build();

        toShootR3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.000, 59.000),
                                new Pose(85.000, 80.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(50.5))
                .build();

        toCollect1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 80.000),
                                new Pose(126.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootC1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126.000, 84.000),
                                new Pose(90.000, 110.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(31))
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
        targetVelocity = Util.getFlywheelVelocityFromDistance(distance)+20;
    }

    private void autoHood() {
        double distance = filteredDistance;
        HoodPos = Util.getHoodPositionFromDistance(distance);
    }

}