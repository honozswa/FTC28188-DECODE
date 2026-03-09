package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Auto Test", group = "Test")
@Configurable
public class AutoTest extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;
    private Paths paths;
    private final Pose startPose = new Pose(36, 135.5, Math.toRadians(180)); // Start Pose of our robot.

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // init other mech

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {

        follower.update();

        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(paths.Shoot1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Collect2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot2, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateCollect1);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot3);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateCollect2);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot4);
                    setPathState(7);

                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Collect3);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot5);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Collect1);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot6);
                    setPathState(11);

                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }

        return pathState;
    }

    public static class Paths {

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

        public Paths(Follower follower) {

            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(36.000, 135.500),
                            new Pose(48.000, 108.000)))
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
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}