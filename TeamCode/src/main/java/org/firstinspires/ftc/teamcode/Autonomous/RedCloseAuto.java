package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.mechanism.ShooterV6;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Red Close Auto")
@Configurable
public class RedCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    // ------ Flywheel Setup ------- //
    private ShooterV6 shooter = new ShooterV6();
    private boolean shotsTriggered = false;
    private double targetVelocity = 1250;
    private enum PathState {
        toShoot1,
        toCollect2,
        toShoot2,
        toRampCollect,
        toShoot3,
        toCollect1,
        toShoot4,
        toCollect3,
        toShoot5,
        Finished,
        toPark,
        Idle
    }
    PathState pathState;

    @Override
    public void init() {
        pathState = PathState.toShoot1;
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseConstant.RedCloseAutoStartPose);

        // init other mech
        shooter.init(hardwareMap);
        shooter.setHood(0.55);

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
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState.toString());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case toShoot1:
                shooter.fullBall();
                shooter.flywheelOn(targetVelocity);
                follower.followPath(Shoot1, true);
                setPathState(PathState.toCollect2);
                break;

            case toCollect2:
                if (!follower.isBusy()) {
                    if (!shotsTriggered && shooter.getFlywheelVel1() > targetVelocity - 100) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(Collect2);
                        setPathState(PathState.toShoot2);
                    }
                }
                break;

            case toShoot2:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, true);
                    setPathState(PathState.toRampCollect);
                }
                break;

            case toRampCollect:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(RampCollect);
                        setPathState(PathState.toShoot3);
                    }
                }
                break;

            case toShoot3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(Shoot3, true);
                    setPathState(PathState.toCollect1);
                }
                break;

            case toCollect1:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(Collect1);
                        setPathState(PathState.toShoot4);
                    }
                }
                break;

            case toShoot4:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot4, true);
                    setPathState(PathState.toCollect3);

                }
                break;

            case toCollect3:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(Collect3);
                        setPathState(PathState.toShoot5);
                    }
                }
                break;

            case toShoot5:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot5, true);
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
                    follower.followPath(Park, true);
                    setPathState(PathState.Idle);
                }
                break;
        }
    }

    // PathChain
    public PathChain Shoot1, Collect2, Shoot2, RampCollect, Shoot3, Collect1, Shoot4, Collect3, Shoot5, Park;
    public void buildPaths() {

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144-36.000, 135.500),
                        new Pose(144-48.000, 100.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        Collect2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144-48.000, 100.000),
                        new Pose(144-51.000, 48.000),
                        new Pose(144-25.000, 62.000),
                        new Pose(144-9.500, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144-9.500, 60.000),
                        new Pose(144-46.000, 64.000),
                        new Pose(144-50.000, 85.000),
                        new Pose(144-48.000, 100.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        RampCollect = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144-48.000, 100.000),
                        new Pose(144-53.000, 63.000),
                        new Pose(144-17.000, 62.000),
                        new Pose(144-9.000, 68.000),
                        new Pose(144-11.000, 53.000)))
                .setConstantHeadingInterpolation(Math.toRadians(40))
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144-11.000, 53.000),
                        new Pose(144-52.000, 70.000),
                        new Pose(144-48.000, 100.000)))
                .setConstantHeadingInterpolation(Math.toRadians(40))
                .build();

        Collect1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144-48.000, 100.000),
                        new Pose(144-44.000, 78.000),
                        new Pose(144-33.000, 86.000),
                        new Pose(144-15.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144-15.000, 84.000),
                        new Pose(144-48.000, 100.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        Collect3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144-48.000, 100.000),
                        new Pose(144-45.000, 27.000),
                        new Pose(144-53.000, 37.000),
                        new Pose(144-9.000, 35.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Shoot5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144-9.000, 35.000),
                        new Pose(144-48.000, 100.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144-48.000, 100.000),
                        new Pose(144-20.000, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

}