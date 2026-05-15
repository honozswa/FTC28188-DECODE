package org.firstinspires.ftc.teamcode.TeleOp.OutOfDated;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.CameraConstant;
import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.mechanism.Util;
import org.firstinspires.ftc.teamcode.mechanism.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//@TeleOp(name = "Blue-Mode")
public class BlueMode extends LinearOpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotorEx shootMotor, shootMotor2;
    DcMotor intakeMotor;
    Servo HoodServo, HoodServo2, GateServo;
    Follower follower;
    private final Webcam webcam = new Webcam();

    // Drivetrain
    double forward = 0, strafe = 0, rotate = 0;

    // Gate
    final double ClosePos = ShooterConstant.closePos;
    final double OpenPos = ShooterConstant.openPos;

    // Flywheel Vel
    double targetVelocity = 0;
    boolean autoShooterEnabled = false;

    // Intake
    private boolean intakeOn = false;

    // Hood
    double HoodPosition1 = ShooterConstant.minServoPos2;
    double HoodPosition2 = 1 - HoodPosition1;
    boolean autoHoodEnabled = false;
    enum ShooterMode { CLOSE, MID, FAR }
    ShooterMode currentMode = ShooterMode.CLOSE;

    // Pose
    private static final Pose GOAL = PoseConstant.BLUE_GOAL;
    boolean autoDriving = false;
    double filteredDistance = 36;

    // Camera Variables
    double CamError = 0;
    double lastCamError = 0;
    double curTime = 0;
    double lastTime = 0;

    @Override
    public void runOpMode() {

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseConstant.BlueAutoStartPose);

        webcam.init(hardwareMap, telemetry);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        HoodServo = hardwareMap.get(Servo.class, "HoodServo");
        HoodServo2 = hardwareMap.get(Servo.class, "HoodServo2");
        GateServo = hardwareMap.get(Servo.class, "GateServo");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);

        HoodServo.setPosition(HoodPosition1);
        HoodServo2.setPosition(HoodPosition2);
        GateServo.setPosition(ClosePos);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        resetRuntime();
        curTime = getRuntime();

        while (opModeIsActive()) {

            follower.update();
            webcam.update();
            AprilTagDetection id = webcam.getTagBySpecificId(CameraConstant.BlueAprilTagsId);

            if (gamepad2.xWasPressed()) {
                autoShooterEnabled = !autoShooterEnabled;
            }
            if (gamepad2.yWasPressed()) {
                autoHoodEnabled = !autoHoodEnabled;
            }

            updateDistance();
            updateHoodMode();
            autoFlywheel();
            autoHood();

            /* =====================
               AUTO DRIVE
            ===================== */
            if(gamepad1.left_bumper && !autoDriving) {
                goToShootPose();
                autoDriving = true;
            }

            /* =====================
               DRIVER OVERRIDE
            ===================== */
            if(autoDriving && driverOverride()) {
                follower.breakFollowing();
                stopDrive();
                setBrake();
                autoDriving = false;
            }

            /* =====================
               DRIVE CONTROL
            ===================== */
            if(autoDriving) {
                follower.update();
                if(!follower.isBusy()) {
                    follower.breakFollowing();
                    stopDrive();
                    autoDriving = false;
                    setBrake();
                }
            } else {
                manualDrive();
                FusionAim(id);
                applyDrive();
            }

            /* =====================
               SUBSYSTEMS
            ===================== */
            subSystem();

            // =======================
            // Telemetry
            // =======================
            telemetry.addData("Shooter1 Power:", shootMotor.getPower());
            telemetry.addData("Shooter2 Power:", shootMotor2.getPower());
            telemetry.addData("Intake Power:", intakeMotor.getPower());
            telemetry.addData("Target Vel:", targetVelocity);
            telemetry.addData("Vel1:", shootMotor.getVelocity());
            telemetry.addData("Vel2:", shootMotor2.getVelocity());
            telemetry.addData("Hood0", HoodPosition1);
            telemetry.addData("Hood1", HoodPosition2);
            telemetry.addData("Distance", getDistanceToGoal(follower.getPose()));
            telemetry.update();
        }
    }

    public double getAngleToGoal(Pose robotPose) {

        double dx = GOAL.getX() - robotPose.getX();
        double dy = GOAL.getY() - robotPose.getY();

        return Math.atan2(dy, dx);
    }

    public void goToShootPose() {

        PathChain shootPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        PoseConstant.BLUE_SHOOT_POSE
                ))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        PoseConstant.BLUE_SHOOT_POSE.getHeading()
                )
                .build();

        follower.followPath(shootPath);
    }

    public boolean driverOverride() {
        return Math.abs(gamepad1.left_stick_x) > 0.15 ||
                Math.abs(gamepad1.left_stick_y) > 0.15 ||
                Math.abs(gamepad1.right_stick_x) > 0.15;
    }

    public void setBrake() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void manualDrive() {
        strafe = gamepad1.left_stick_x;
        forward = -gamepad1.left_stick_y;
        rotate = gamepad1.right_stick_x / 1.8;
    }

    public void subSystem() {
        // =======================
        // Flywheel
        // =======================

        shootMotor.setVelocity(targetVelocity);
        shootMotor2.setVelocity(targetVelocity);

        // =======================
        // Intake toggle (A) + Gate (RB)
        // =======================
        if (gamepad1.aWasPressed()) {
            intakeOn = !intakeOn;
        }

        if (gamepad1.right_bumper) {
            GateServo.setPosition(OpenPos);
            intakeMotor.setPower(1);
        } else if (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
            intakeMotor.setPower(1);
            GateServo.setPosition(ClosePos);
        } else if (gamepad1.x) {
            GateServo.setPosition(OpenPos);
            intakeMotor.setPower(0.6);
        } else {
            intakeMotor.setPower(intakeOn ? 0.5 : 0.0);
            GateServo.setPosition(ClosePos);
        }

        // =======================
        // Hood
        // =======================
        HoodServo.setPosition(HoodPosition1);
        HoodServo2.setPosition(HoodPosition2);

    }

    public void FusionAim(AprilTagDetection id) {
        if (gamepad1.left_trigger > 0.5) {
            if (id != null) {
                CamError = CameraConstant.goalX - id.ftcPose.bearing;

                if (Math.abs(CamError) < CameraConstant.angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = CamError * CameraConstant.kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((CamError - lastCamError) / dT) * CameraConstant.kD;

                    rotate = Range.clip(pTerm + dTerm, -0.4,0.4);

                    lastCamError = CamError;
                    lastTime = curTime;
                }
            } else {
                Pose robotPose = follower.getPose();
                double targetHeading = getAngleToGoal(robotPose);
                double error = Util.angleWrap(robotPose.getHeading() - targetHeading);
                double kP = 1;
                rotate = Range.clip(error * kP, -1, 1);
            }
        }
    }

    public void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void applyDrive() {

        double lfPower = forward + strafe + rotate;
        double rfPower = forward - strafe - rotate;
        double lbPower = forward - strafe + rotate;
        double rbPower = forward + strafe - rotate;

        leftFront.setPower(Range.clip(lfPower, -1, 1));
        rightFront.setPower(Range.clip(rfPower, -1, 1));
        leftBack.setPower(Range.clip(lbPower, -1, 1));
        rightBack.setPower(Range.clip(rbPower, -1, 1));
    }

    public double getDistanceToGoal(Pose robotPose) {

        double dx = GOAL.getX() - robotPose.getX();
        double dy = GOAL.getY() - robotPose.getY();

        return Math.sqrt(dx*dx + dy*dy);
    }

    public void updateDistance() {
        Pose robotPose = follower.getPose();
        double rawDistance = getDistanceToGoal(robotPose);
        filteredDistance = 0.8 * filteredDistance + 0.2 * rawDistance;
    }

    public void updateHoodMode() {
        double d = filteredDistance;

        switch (currentMode) {
            case CLOSE:
                if (d > ShooterConstant.closeRange)
                    currentMode = ShooterMode.MID;
                break;

            case MID:
                if (d < ShooterConstant.closeRange)
                    currentMode = ShooterMode.CLOSE;
                else if (d > ShooterConstant.midRange)
                    currentMode = ShooterMode.FAR;
                break;

            case FAR:
                if (d < ShooterConstant.midRange)
                    currentMode = ShooterMode.MID;
                break;
        }
    }

    public void autoFlywheel() {

        if (!autoShooterEnabled) {
            targetVelocity = 0;
        } else {

            double distance = filteredDistance;

            if (currentMode == ShooterMode.CLOSE) {
                double velocity = Range.clip(Util.getFlywheelVelocityFromDistanceClose(distance), ShooterConstant.minTicks, ShooterConstant.maxTicks);
                targetVelocity = 0.8 * targetVelocity + 0.2 * velocity;
            }
            else if (currentMode == ShooterMode.MID) {
                double velocity = Range.clip(Util.getFlywheelVelocityFromDistanceMid(distance), ShooterConstant.minTicks, ShooterConstant.maxTicks);
                targetVelocity = 0.8 * targetVelocity + 0.2 * velocity;
            }
            else {
                double velocity = Range.clip(Util.getFlywheelVelocityFromDistanceFar(distance), ShooterConstant.minTicks, ShooterConstant.maxTicks);
                targetVelocity = 0.8 * targetVelocity + 0.2 * velocity;
            }

            telemetry.addData("AutoVel", targetVelocity);
            telemetry.update();
        }
    }

    public void autoHood() {

        if (!autoHoodEnabled) {

            if (gamepad2.dpad_up) {
                HoodPosition1 += 0.01;
            }
            if (gamepad2.dpad_down) {
                HoodPosition1 -= 0.01;
            }
            HoodPosition1 = Range.clip(HoodPosition1, ShooterConstant.minServoPos2, ShooterConstant.maxServoPos1);

        } else {

            if (currentMode == ShooterMode.CLOSE) {
                double target = ShooterConstant.CloseModePos;
                HoodPosition1 = 0.8 * HoodPosition1 + 0.2 * target;
            }
            else if (currentMode == ShooterMode.MID) {
                double target = ShooterConstant.MidModePos;
                HoodPosition1 = 0.8 * HoodPosition1 + 0.2 * target;
            }
            else {
                double target = ShooterConstant.FarModePos;
                HoodPosition1 = 0.8 * HoodPosition1 + 0.2 * target;
            }
        }
        HoodPosition2 = 1 - HoodPosition1;

    }

}
