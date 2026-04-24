package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Autonomous.ShotMechanicTest;
import org.firstinspires.ftc.teamcode.Constants.CameraConstant;
import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.mechanism.Util;
import org.firstinspires.ftc.teamcode.mechanism.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name = "Blue-TrajectoryTest")
public class BlueTrajectoryTest extends LinearOpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotorEx shootMotor, shootMotor2;
    DcMotor intakeMotor;
    Servo HoodServo, HoodServo2, GateServo;
    Follower follower;
    private final Webcam webcam = new Webcam();

    // Drivetrain
    double targetX = 0, targetY = 0, targetTurn = 0;
    double currentX = 0, currentY = 0, currentTurn = 0;
    double rampRate = 1;

    // Gate
    final double ClosePos = ShooterConstant.closePos;
    final double OpenPos = ShooterConstant.openPos;

    // Flywheel Vel
    double targetVelocity = 0;
    double velOffset = ShooterConstant.FlywheelOffset;

    // Intake
    private boolean intakeOn = false;

    // Hood
    double HoodPosition1 = ShooterConstant.minServoPos2;
    double HoodPosition2 = 1 - HoodPosition1;
    double lastHoodPos = 0;
    double lastLaunchAngle = -999;

    // Pose
    private static final Pose GOAL = PoseConstant.BLUE_GOAL;
    boolean autoDriving = false;
    long lastShooterUpdate = 0;
    double distanceFiltered = 60; // starting guess (any reasonable distance)

    boolean autoShooterEnabled = false;

    // Camera Variables
    double CamError = 0;
    double lastCamError = 0;
    double curTime = 0;
    double lastTime = 0;

    @Override
    public void runOpMode() {

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(36, 135.5, Math.toRadians(180)));

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

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(150 , 0, 0, 17.0390);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

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

//            autoShooter();
            predictTrajectory();

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
    private double rampTowardsTarget(double current, double target, double rate) {
        double delta = target - current;
        if (Math.abs(delta) > rate) {
            return current + Math.signum(delta) * rate;
        } else {
            return target;
        }
    }

    public double getAngleToGoal(Pose robotPose) {

        double dx = GOAL.getX() - robotPose.getX();
        double dy = GOAL.getY() - robotPose.getY();

        return Math.atan2(dy, dx);
    }

    public double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
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
        targetX = gamepad1.left_stick_x;
        targetY = -gamepad1.left_stick_y;
        targetTurn = gamepad1.right_stick_x / 1.8;

        currentX = rampTowardsTarget(currentX, targetX, rampRate);
        currentY = rampTowardsTarget(currentY, targetY, rampRate);
        currentTurn = rampTowardsTarget(currentTurn, targetTurn, rampRate);
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
            intakeMotor.setPower(0.65);
        } else if (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
            intakeMotor.setPower(1);
            GateServo.setPosition(ClosePos);
        } else if (gamepad1.x) {
            GateServo.setPosition(OpenPos);
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(intakeOn ? 0.5 : 0.0);
            GateServo.setPosition(ClosePos);
        }

        // =======================
        // Hood
        // =======================
        if (gamepad2.dpad_up) {
            HoodPosition1 += 0.01;
        }
        if (gamepad2.dpad_down) {
            HoodPosition1 -= 0.01;
        }
        HoodPosition1 = Range.clip(HoodPosition1, ShooterConstant.minServoPos2, ShooterConstant.minServoPos2);
        HoodPosition2 = 1 - HoodPosition1;

        HoodServo.setPosition(HoodPosition1);
        HoodServo2.setPosition(HoodPosition2);

    }

    public void aimBot() {
        if (gamepad1.left_trigger > 0.5) {
            Pose robotPose = follower.getPose();
            double targetHeading = getAngleToGoal(robotPose);
            double error = angleWrap(robotPose.getHeading() - targetHeading);
            double kP = 1;
            currentTurn = Range.clip(error * kP, -1, 1);
            // stop oscillation
            if (Math.abs(error) < Math.toRadians(0.05)) {
                currentTurn = 0;
            }
        }
    }

    public void FusionAim(AprilTagDetection id) {
        if (gamepad1.left_trigger > 0.5) {
            if (id != null) {
                CamError = CameraConstant.goalX - id.ftcPose.bearing;

                if (Math.abs(CamError) < CameraConstant.angleTolerance) {
                    currentTurn = 0;
                } else {
                    double pTerm = CamError * CameraConstant.kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((CamError - lastCamError) / dT) * CameraConstant.kD;

                    currentTurn = Range.clip(pTerm + dTerm, -0.4,0.4);

                    lastCamError = CamError;
                    lastTime = curTime;
                }
            } else {
                Pose robotPose = follower.getPose();
                double targetHeading = getAngleToGoal(robotPose);
                double error = angleWrap(robotPose.getHeading() - targetHeading);
                double kP = 1;
                currentTurn = Range.clip(error * kP, -1, 1);
                // stop oscillation
//                if (Math.abs(error) < Math.toRadians(5)) {
//                    currentTurn = 0;
//                }
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

        double lfPower = currentY + currentX + currentTurn;
        double rfPower = currentY - currentX - currentTurn;
        double lbPower = currentY - currentX + currentTurn;
        double rbPower = currentY + currentX - currentTurn;

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

    public double getFlywheelVelocity(double distance) {
        return 776.27374 * Math.pow(1.00522, distance) + velOffset;
    }

    public double getHoodPosition(double distance) {
        return 0.00000113108 * Math.pow(distance, 3)
                - 0.000328854 * Math.pow(distance, 2)
                + 0.0321784 * distance
                - 0.750827;
    }

    public void autoShooter() {

        if (!autoShooterEnabled) {
            targetVelocity = 0;
        } else if (autoShooterEnabled) {

            Pose robotPose = follower.getPose();

            double rawDistance = getDistanceToGoal(robotPose);
            distanceFiltered = 0.8 * distanceFiltered
                    + 0.2 * rawDistance;

            // prevent regression explosion
            rawDistance = Range.clip(rawDistance, 0, 300);

            double velocity = getFlywheelVelocity(distanceFiltered);
            double hood = getHoodPosition(distanceFiltered);

            velocity = Range.clip(velocity, ShooterConstant.minTicks, ShooterConstant.maxTicks);
            hood = Range.clip(hood, ShooterConstant.minServoPos2, ShooterConstant.maxServoPos1);

            if (System.currentTimeMillis() - lastShooterUpdate > 100) {
                targetVelocity = velocity;
                lastShooterUpdate = System.currentTimeMillis();
            }

            HoodPosition1 = hood;
            HoodPosition2 = 1 - hood;

            HoodServo.setPosition(HoodPosition1);
            HoodServo2.setPosition(HoodPosition2);

            telemetry.addData("Raw Distance", rawDistance);
            telemetry.addData("Filtered Distance", distanceFiltered);
            telemetry.addData("Auto Vel", velocity);
            telemetry.addData("Auto Hood", hood);
        }
    }

    private void predictTrajectory() {

        if (!autoShooterEnabled) {
            targetVelocity = 0;
        } else if (autoShooterEnabled) {

            double g = ShooterConstant.g;
            double rawX = getDistanceToGoal(follower.getPose()) * 0.0254; // Convert Inches to meters
            double y = ShooterConstant.entryHeight;
            double a = ShooterConstant.entryAngle;

            distanceFiltered = 0.9 * distanceFiltered + 0.1 * rawX;

            double x = distanceFiltered;

            if (x < 0.1) return;

            double launchAngle = MathFunctions.clamp(Math.atan((2 * y / x) - Math.tan(a)), ShooterConstant.minAngle, ShooterConstant.maxAngle);

            double angleThreshold = Math.toRadians(1.0); // 1 degree

            if (Math.abs(launchAngle - lastLaunchAngle) < angleThreshold) {
                return;
            }
            lastLaunchAngle = launchAngle;

            double denominator = (x * Math.tan(launchAngle) - y);
            if (denominator <= 0.01) return;

            double launchVel = Math.sqrt((g * x * x) / (2 * Math.cos(launchAngle) * Math.cos(launchAngle) * denominator));

            double FlywheelVel = Range.clip(Util.getFlywheelVelocityFromV0(launchVel), 900, 1700);
            double hoodPos = Range.clip(Util.getHoodServoPosFromAngle(launchAngle), ShooterConstant.minServoPos2, ShooterConstant.maxServoPos1);

            if (System.currentTimeMillis() - lastShooterUpdate > 100) {
                targetVelocity = FlywheelVel;
                lastShooterUpdate = System.currentTimeMillis();
            }

            HoodPosition1 = hoodPos;
            HoodPosition2 = 1 - hoodPos;

            HoodServo.setPosition(HoodPosition1);
            HoodServo2.setPosition(HoodPosition2);
        }
    }

}
