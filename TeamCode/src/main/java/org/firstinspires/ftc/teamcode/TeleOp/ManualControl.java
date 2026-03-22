package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ManualControl")
public class ManualControl extends LinearOpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotorEx shootMotor, shootMotor2, gateMotor;
    DcMotor intakeMotor;
    Servo HoodServo, HoodServo2, GateServo;
    Follower follower;

    // Drivetrain
    double targetX = 0, targetY = 0, targetTurn = 0;
    double currentX = 0, currentY = 0, currentTurn = 0;
    double rampRate = 1;

    // Gate
    final double ClosePos = 0.3;
    final double OpenPos = 0.0;

    // Flywheel Vel
    double ZERO_VELOCITY = 0;
    double CLOSE_VELOCITY = 1100;
    double MID_VELOCITY = 1700;
    double targetVelocity = 0;
    final double VELOCITY_STEP = 50;
    boolean lastUp = false;
    boolean lastDown = false;

    // Intake
    private boolean intakeOn = false;

    // Hood
    double HoodPosition1 = 0.3;
    double HoodPosition2 = 1 - HoodPosition1;

    // Pose
    public static final Pose GOAL = new Pose(-5 , 138.67);
    public static final Pose SHOOT_POSE = new Pose(55, 88, Math.toRadians(139.5));
    boolean autoDriving = false;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        gateMotor = hardwareMap.get(DcMotorEx.class, "gateMotor");
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
        gateMotor.setDirection(DcMotor.Direction.REVERSE);

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(150, 0, 0.01, 17.0390);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        HoodServo.setPosition(HoodPosition1);
        HoodServo2.setPosition(HoodPosition2);
        GateServo.setPosition(ClosePos);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(36, 135.5, Math.toRadians(180)));

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

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
                aimBot();
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
                        SHOOT_POSE
                ))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        SHOOT_POSE.getHeading()
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
        if (gamepad1.xWasPressed()) {
            targetVelocity = CLOSE_VELOCITY;
        }
        if (gamepad1.yWasPressed()) {
            targetVelocity = MID_VELOCITY;
        }
        if (gamepad1.bWasPressed()) {
            targetVelocity = ZERO_VELOCITY;
        }
        if (gamepad1.dpad_up && !lastUp) {
            targetVelocity += VELOCITY_STEP;
        }
        if (gamepad1.dpad_down && !lastDown) {
            targetVelocity -= VELOCITY_STEP;
        }

        targetVelocity = Math.max(0, targetVelocity);

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

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
            shootMotor.setVelocity(targetVelocity + 200);
            shootMotor2.setVelocity(targetVelocity + 200);
            intakeMotor.setPower(0.65);
            gateMotor.setPower(1);
        } else if (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
            intakeMotor.setPower(1);
            GateServo.setPosition(ClosePos);
        } else if (gamepad1.dpad_left) {
            GateServo.setPosition(OpenPos);
            intakeMotor.setPower(1);
            gateMotor.setPower(0.6);
        } else {
            gateMotor.setPower(intakeOn ? 0.5 : 0.0);
            GateServo.setPosition(ClosePos);
            intakeMotor.setPower(0.0);
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

        HoodPosition1 = Range.clip(HoodPosition1, 0.15, 0.4);

        HoodPosition2 = 1 - HoodPosition1;

        HoodServo.setPosition(HoodPosition1);
        HoodServo2.setPosition(HoodPosition2);
    }

    public void aimBot() {
        double turnPower = currentTurn;

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

}
