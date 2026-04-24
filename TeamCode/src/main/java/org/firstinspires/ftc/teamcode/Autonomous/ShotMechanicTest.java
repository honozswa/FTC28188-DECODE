package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.mechanism.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Shot's Mechanic Test", group = "Test")
@Configurable
public class ShotMechanicTest extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    // ------ Flywheel Setup ------- //
    private Shooter shooter = new Shooter();
    private boolean shotsTriggered = false;
    private double targetVelocity = 1150;

    private enum PathState {
        toShoot1,
        toCollect2,
        toShoot2,
        toGateCollect1,
        toShoot3,
        toGateCollect2,
        toShoot4,
        toCollect3,
        toShoot5,
        toCollect1,
        toShoot6,
        Finished,
        Idle
    }
    PathState pathState;
    private final Pose startPose = new Pose(36, 135.5, Math.toRadians(180)); // Start Pose of our robot.

    @Override
    public void init() {
        pathState = PathState.toShoot1;
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // init other mech
        shooter.init(hardwareMap);

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
                shooter.intakeOn(0.8);
                shooter.flywheelOn(targetVelocity);
                follower.followPath(Shoot1, true);
                setPathState(PathState.Finished);
                break;

            case toCollect2:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
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
                    setPathState(PathState.toGateCollect1);
                }
                break;

            case toGateCollect1:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(GateCollect1);
                        setPathState(PathState.toShoot3);
                    }
                }
                break;

            case toShoot3:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot3, true);
                    setPathState(PathState.toCollect3);
                }
                break;

            case toGateCollect2:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShot();
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        follower.followPath(GateCollect2);
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
                        setPathState(PathState.toShoot6);
                    }
                }
                break;

            case toShoot6:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot6, true);
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
    public PathChain Shoot1;
    public PathChain Collect2;
    public PathChain Shoot2;
    public PathChain GateCollect1;
    public PathChain Shoot3;
    public PathChain GateCollect2;
    public PathChain Shoot4;
    public PathChain Collect3;
    public PathChain Shoot5;
    public PathChain Collect1;
    public PathChain Shoot6;

    public void buildPaths() {

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                                new Pose(36.000, 135.500),
                                new Pose(48.000, 100.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        Collect2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(48.000, 108.000),
                        new Pose(51.650, 44.794),
                        new Pose(31.444, 64.000),
                        new Pose(9.486, 59.290)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(9.486, 59.290),
                        new Pose(58.341, 56.907),
                        new Pose(47.350, 102.215),
                        new Pose(47.776, 108.112)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        GateCollect1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(47.776, 108.112),
                        new Pose(51.794, 58.037),
                        new Pose(13.121, 61.963)))
                .setConstantHeadingInterpolation(Math.toRadians(140))
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(13.121, 61.963),
                        new Pose(57.551, 62.850),
                        new Pose(47.776, 108.112)))
                .setConstantHeadingInterpolation(Math.toRadians(140))
                .build();

        GateCollect2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(47.776, 108.112),
                        new Pose(51.794, 58.037),
                        new Pose(13.121, 61.963)))
                .setConstantHeadingInterpolation(Math.toRadians(140))
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(13.121, 61.963),
                        new Pose(57.551, 62.850),
                        new Pose(47.776, 108.112)))
                .setConstantHeadingInterpolation(Math.toRadians(140))
                .build();

        Collect3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(47.776, 108.112),
                        new Pose(51.187, 16.285),
                        new Pose(40.561, 39.818),
                        new Pose(9.047, 35.019)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shoot5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(9.047, 35.019),
                        new Pose(55.897, 71.528),
                        new Pose(47.776, 108.112)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        Collect1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(47.776, 108.112),
                        new Pose(53.453, 80.995),
                        new Pose(15.243, 83.654)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shoot6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15.243, 83.654),
                        new Pose(47.776, 108.112)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();
    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

}