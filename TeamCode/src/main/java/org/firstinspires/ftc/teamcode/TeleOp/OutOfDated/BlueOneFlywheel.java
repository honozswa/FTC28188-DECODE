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


//@TeleOp(name = "Blue-OneFlywheel")
public class BlueOneFlywheel extends LinearOpMode {

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
    double ZERO_VELOCITY = ShooterConstant.ZeroVel;
    double CLOSE_VELOCITY = ShooterConstant.CloseVel;
    double MID_VELOCITY = ShooterConstant.MidVel;
    double targetVelocity = 0;
    final double VELOCITY_STEP = ShooterConstant.VelStep;

    // Intake
    private boolean intakeOn = false;

    // Hood
    double HoodPosition1 = ShooterConstant.minServoPos2;
    double HoodPosition2 = 1 - HoodPosition1;

    // Pose
    public static final Pose GOAL = PoseConstant.BLUE_GOAL;
    public static final Pose SHOOT_POSE = PoseConstant.BLUE_SHOOT_POSE;
    boolean autoDriving = false;

    // Camera Variables
    int AprilTagsId = CameraConstant.BlueAprilTagsId;
    double kP = CameraConstant.kP;
    double CamError = 0;
    double lastCamError = 0;
    double goalX = CameraConstant.goalX;
    double angleTolerance = CameraConstant.angleTolerance;
    double kD = CameraConstant.kD;
    double curTime = 0;
    double lastTime = 0;

    @Override
    public void runOpMode() {

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
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);

        HoodServo.setPosition(HoodPosition1);
        HoodServo2.setPosition(HoodPosition2);
        GateServo.setPosition(ClosePos);

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseConstant.BlueAutoStartPose);

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

        while (opModeIsActive()) {

            follower.update();
            webcam.update();
            AprilTagDetection id = webcam.getTagBySpecificId(AprilTagsId);

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
//            telemetry.addData("Shooter2 Power:", shootMotor2.getPower());
            telemetry.addData("Intake Power:", intakeMotor.getPower());
            telemetry.addData("Target Vel:", targetVelocity);
            telemetry.addData("Vel1:", shootMotor.getVelocity());
//            telemetry.addData("Vel2:", shootMotor2.getVelocity());
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
        strafe = gamepad1.left_stick_x;
        forward = -gamepad1.left_stick_y;
        rotate = gamepad1.right_stick_x / 1.8;
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
        if (gamepad1.dpadUpWasPressed()) {
            targetVelocity += VELOCITY_STEP;
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetVelocity -= VELOCITY_STEP;
        }

        targetVelocity = Math.max(0, targetVelocity);

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
        } else if (gamepad1.dpad_left) {
            GateServo.setPosition(OpenPos);
            intakeMotor.setPower(0.6);
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

        HoodPosition1 = Range.clip(HoodPosition1, ShooterConstant.minServoPos2, ShooterConstant.maxServoPos1);

        HoodPosition2 = 1 - HoodPosition1;

        HoodServo.setPosition(HoodPosition1);
        HoodServo2.setPosition(HoodPosition2);
    }

    public void FusionAim(AprilTagDetection id) {
        if (gamepad1.left_trigger > 0.5) {
            if (id != null) {
                CamError = goalX - id.ftcPose.bearing;

                if (Math.abs(CamError) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = CamError * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((CamError - lastCamError) / dT) * kD;

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

}
