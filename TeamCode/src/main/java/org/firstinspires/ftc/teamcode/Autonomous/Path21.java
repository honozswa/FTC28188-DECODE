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

@Autonomous(name = "Test-21shotsPath", group = "Test")
@Configurable
public class Path21 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    private enum PathState {
        toShoot1,
        toCollect2,
        toShoot2,
        toRampCollect1,
        toShoot3,
        toRampCollect2,
        toShoot4,
        toRampCollect3,
        toShoot5,
        toCollect1,
        toShoot6,
        toCollect3,
        toShoot7,

        Finished,
        toPark,
        Idle
    }
    PathState pathState;

    // Pose
    private final Pose startPose = PoseConstant.BlueAutoStartPose;

    @Override
    public void init() {
        pathState = PathState.toShoot1;
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

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
                follower.followPath(Shoot1, true);
                setPathState(PathState.toCollect2);
                break;

            case toCollect2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(Collect2);
                    setPathState(PathState.toShoot2);
                }
                break;

            case toShoot2:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot2, true);
                    setPathState(PathState.toRampCollect1);
                }
                break;

            case toRampCollect1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(RampCollect1, true);
                    setPathState(PathState.toShoot3);
                }
                break;

            case toShoot3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(Shoot3, true);
                    setPathState(PathState.toRampCollect2);
                }
                break;

            case toRampCollect2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(RampCollect2, true);
                    setPathState(PathState.toShoot4);
                }
                break;

            case toShoot4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(Shoot4, true);
                    setPathState(PathState.toCollect3);

                }
                break;

            case toRampCollect3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(RampCollect3, true);
                    setPathState(PathState.toShoot5);
                }
                break;

            case toShoot5:
                if (!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(Shoot5, true);
                    setPathState(PathState.toCollect1);
                }
                break;

            case toCollect1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(Collect1, true);
                    setPathState(PathState.toShoot6);
                }
                break;

            case toShoot6:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot6, true);
                    setPathState(PathState.toCollect3);
                }
                break;

            case toCollect3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(Collect3, true);
                    setPathState(PathState.toShoot7);
                }
                break;

            case toShoot7:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot7, true);
                    setPathState(PathState.Finished);
                }
                break;

            case Finished:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(PathState.toPark);
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
    public PathChain Shoot1, Collect2, Shoot2, RampCollect1, Shoot3, RampCollect2, Shoot4, RampCollect3, Shoot5, Collect1, Shoot6, Collect3, Shoot7, Park;
    public void buildPaths() {

        Shoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(36.000, 135.000),
                                new Pose(50.000, 80.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        Collect2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 80.000),
                                new Pose(48.000, 54.000),
                                new Pose(10.000, 59.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setNoDeceleration()
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.000, 59.000),
                                new Pose(27.393, 58.939),
                                new Pose(53.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(223))
                .setNoDeceleration()
                .build();

        RampCollect1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.000, 81.000),
                                new Pose(31.100, 60.500),
                                new Pose(12.692, 59.224)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(223), Math.toRadians(150))
                .setNoDeceleration()
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.692, 59.224),
                                new Pose(32.210, 62.262),
                                new Pose(53.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(223))
                .setNoDeceleration()
                .build();

        RampCollect2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.000, 81.000),
                                new Pose(31.300, 60.500),
                                new Pose(12.692, 59.224)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(223), Math.toRadians(150))
                .setNoDeceleration()
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.692, 59.224),
                                new Pose(32.400, 62.262),
                                new Pose(53.100, 81.100)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(223))
                .setNoDeceleration()
                .build();

        RampCollect3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.100, 81.100),
                                new Pose(31.300, 60.400),
                                new Pose(12.692, 59.224)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(223), Math.toRadians(150))
                .setNoDeceleration()
                .build();

        Shoot5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.692, 59.224),
                                new Pose(32.304, 63.421),
                                new Pose(53.673, 83.692)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(223))
                .setNoDeceleration()
                .build();

        Collect1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(53.673, 83.692),
                                new Pose(17.028, 83.804)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setNoDeceleration()
                .build();

        Shoot6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(17.028, 83.804),
                                new Pose(48.645, 84.028)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setNoDeceleration()
                .build();

        Collect3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.645, 84.028),
                                new Pose(54.061, 28.720),
                                new Pose(12.654, 36.832)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        Shoot7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.654, 36.832),
                                new Pose(56.364, 79.393)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setNoDeceleration()
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.364, 79.393),
                                new Pose(48.374, 71.673)
                        )
                )
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

    }


    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

//        shotsTriggered = false;
    }

}