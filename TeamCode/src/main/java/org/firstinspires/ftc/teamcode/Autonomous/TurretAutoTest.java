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

@Autonomous(name = "Shot's Mechanic Test", group = "Test")
@Configurable
public class TurretAutoTest extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;

    // ------ Flywheel Setup ------- //
    private MecanumDrive drive = new MecanumDrive();
    private ShooterV2 shooter = new ShooterV2();
    private Turret turret = new Turret();
    private Limelight3A limelight;
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

    // Pose
    private final Pose startPose = PoseConstant.BlueAutoStartPose;
    private static final Pose GOAL = PoseConstant.BLUE_GOAL;

    // Camera Variables
    double lastErrorCam = 0;
    double lastErrorOdo = 0;

    // Turret
    boolean turretOn = true;
    double turretPower = 0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        pathState = PathState.toShoot1;
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // init other mech
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1); // BlueTag
        limelight.start();

        turret.init(hardwareMap);
        shooter.init(hardwareMap);
        shooter.setHood(0.55);

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
                        new Pose(36.000, 135.500),
                        new Pose(48.000, 100.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        Collect2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(48.000, 100.000),
                        new Pose(51.000, 48.000),
                        new Pose(25.000, 62.000),
                        new Pose(9.500, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(9.500, 60.000),
                        new Pose(46.000, 64.000),
                        new Pose(50.000, 85.000),
                        new Pose(48.000, 100.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        RampCollect = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(48.000, 100.000),
                        new Pose(53.000, 63.000),
                        new Pose(17.000, 62.000),
                        new Pose(9.000, 68.000),
                        new Pose(11.000, 53.000)))
                .setConstantHeadingInterpolation(Math.toRadians(140))
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(11.000, 53.000),
                        new Pose(52.000, 70.000),
                        new Pose(48.000, 100.000)))
                .setConstantHeadingInterpolation(Math.toRadians(140))
                .build();

        Collect1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(48.000, 100.000),
                        new Pose(44.000, 78.000),
                        new Pose(33.000, 86.000),
                        new Pose(15.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15.000, 84.000),
                        new Pose(48.000, 100.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        Collect3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(48.000, 100.000),
                        new Pose(45.000, 27.000),
                        new Pose(53.000, 37.000),
                        new Pose(9.000, 35.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shoot5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(9.000, 35.000),
                        new Pose(48.000, 100.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(48.000, 100.000),
                        new Pose(20.000, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
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
            if (result.isValid()) {
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
                if (Math.abs(error) < TurretConstant.angleTolerance) {
                    turretPower = 0;
                    lastErrorOdo = 0;
                } else {
                    double pTerm = error * TurretConstant.OdokP;
                    double dTerm = 0;
                    if (dt > 0) {
                        dTerm = ((error - lastErrorOdo) / dt) * TurretConstant.OdokD;
                    }
                    lastErrorOdo = error;
                    double FeedForward = TurretConstant.OdokF * follower.getAngularVelocity();
                    turretPower = pTerm + dTerm - FeedForward;
                }
                turret.setPower(Range.clip(turretPower,-TurretConstant.MAX_POWER,TurretConstant.MAX_POWER));

                telemetry.addLine("Odometry in use");
                telemetry.addLine("");

            }
        }
    }

}