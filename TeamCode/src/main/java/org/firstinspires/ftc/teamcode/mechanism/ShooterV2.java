package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class ShooterV2 {
    private DcMotor intakeMotor, outtakeMotor;
    private DcMotorEx shootMotor, shootMotor2;
    private Servo HoodServo, HoodServo2, GateServo;
    private Servo IntakeServo, IntakeServo2;
    DistanceSensor distanceSens = new DistanceSensor();
    ColorSensor colorSensor = new ColorSensor();
    ColorSensor.DetectedColor detectedColor;
    ColorSensor.DetectedColor2 detectedColor2;
    private ElapsedTime stateTimer = new ElapsedTime();
    private enum ShooterState {
        Idle,
        Intake,
        Hold,
        ThreeBall,
        Shot,
        Done,
    }
    private ShooterState shooterState;

    // Gate
    private double ClosePos = ShooterConstant.closePos;
    private double OpenPos = ShooterConstant.openPos;
    private double ShotTime = ShooterConstant.shotTime;
    private boolean shotRequested = false;
    private boolean returnRequested = false;

    // Hood
    double HoodPos = ShooterConstant.minServoPos2;

    // Intake
    private boolean intakeisOn = false;
    private boolean intakeBoost = false;
    private boolean outtakeisOn = false;
    private int ball = 0;
    private boolean lastDetected = false;

    private double downPos = ShooterConstant.intakeServoDownPos;
    private double upPos = ShooterConstant.intakeServoUpPos;
    private double pushPos = ShooterConstant.intakeServoPushPos;
    private double intakeServo2Offset = 0.07;
    private double intakeServoOffsetDuringGame = 0;

    public void init(HardwareMap hwMap) {
        shootMotor = hwMap.get(DcMotorEx.class, "shootMotor");
        shootMotor2 = hwMap.get(DcMotorEx.class, "shootMotor2");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
//        outtakeMotor = hwMap.get(DcMotor.class, "gateMotor");
        HoodServo = hwMap.get(Servo.class, "HoodServo");
        HoodServo2 = hwMap.get(Servo.class, "HoodServo2");
        GateServo = hwMap.get(Servo.class, "GateServo");
        IntakeServo = hwMap.get(Servo.class,"IntakeServo");
        IntakeServo2 = hwMap.get(Servo.class,"IntakeServo2");
        distanceSens.init(hwMap);
        colorSensor.init(hwMap);

        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
//        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
        HoodServo2.setDirection(Servo.Direction.REVERSE);
        IntakeServo2.setDirection(Servo.Direction.REVERSE);

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);

        HoodServo.setPosition(HoodPos);
        HoodServo2.setPosition(HoodPos);
        GateServo.setPosition(ClosePos);
        IntakeServo.setPosition(downPos);
        IntakeServo2.setPosition(downPos+intakeServo2Offset);

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateTimer.reset();
        shooterState = ShooterState.Idle;
    }

    public void update() {
        detectedColor = colorSensor.getDetectedBall();
        boolean lastBallDetected = detectedColor != ColorSensor.DetectedColor.UNKNOWN;

        detectedColor2 = colorSensor.getDetectedBall2();
        boolean secondBallDetected = detectedColor2 != ColorSensor.DetectedColor2.UNKNOWN;

        boolean firstBallDetected = distanceSens.ballDetection();

        switch (shooterState) {
            case Idle:
                IntakeServo.setPosition(upPos+intakeServoOffsetDuringGame);
                IntakeServo2.setPosition(upPos+intakeServo2Offset+intakeServoOffsetDuringGame);
                intakeMotor.setPower(0);
//                outtakeMotor.setPower(0);
//                outtakeMotor.setPower(outtakeisOn ? 1 : 0);
                GateServo.setPosition(ClosePos);
                if (intakeisOn) {
                    shooterState = ShooterState.Intake;
                }
                if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                break;

            case Intake:
                IntakeServo.setPosition(downPos+intakeServoOffsetDuringGame);
                IntakeServo2.setPosition(downPos+intakeServo2Offset+intakeServoOffsetDuringGame);
                intakeMotor.setPower(1);
//                outtakeMotor.setPower(1);
//                outtakeMotor.setPower(outtakeisOn ? 1 : 0);
                GateServo.setPosition(ClosePos);
                if (!intakeisOn) {
                    shooterState = ShooterState.Idle;
                }
//                if (lastBallDetected && !lastDetected) {
//                    ball++;
//                    stateTimer.reset();
//                }
//                if (ball >= 1 && !lastBallDetected) {
//                    shooterState = ShooterState.Hold;
//                }
                if (firstBallDetected && secondBallDetected && lastBallDetected) {
                    stateTimer.reset();
                    shooterState = ShooterState.ThreeBall;
                }
                if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                break;

            case Hold:
                IntakeServo.setPosition(downPos+intakeServoOffsetDuringGame);
                IntakeServo2.setPosition(downPos+intakeServo2Offset+intakeServoOffsetDuringGame);
                intakeMotor.setPower(1);
//                outtakeMotor.setPower(1);
//                outtakeMotor.setPower(outtakeisOn ? 0.5 : 0);
                GateServo.setPosition(ClosePos);
                if (!intakeisOn) {
                    shooterState = ShooterState.Idle;
                }
//                if (lastBallDetected && !lastDetected) {
//                    ball++;
//                    stateTimer.reset();
//                }
//                if (ball == 2) {
//                    outtakeMotor.setPower(1);
//                    if (stateTimer.seconds() > 0.5) {
//                        outtakeMotor.setPower(0.5);
//                    }
//                }
                if (ball >= 3) {
                    if (stateTimer.seconds() > 0.5) {
                        stateTimer.reset();
                        shooterState = ShooterState.ThreeBall;
                    }
                }
                if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                break;

            case ThreeBall:
                intakeMotor.setPower(0);
//                outtakeMotor.setPower(0);
                GateServo.setPosition(ClosePos);
                IntakeServo.setPosition(upPos+intakeServoOffsetDuringGame);
                IntakeServo2.setPosition(upPos+intakeServo2Offset+intakeServoOffsetDuringGame);
                if (returnRequested) {
                    returnRequested = false;
//                    ball = 2;
                    shooterState = ShooterState.Intake;
                }
                if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                break;

            case Shot:
                IntakeServo.setPosition(upPos+intakeServoOffsetDuringGame);
                IntakeServo2.setPosition(upPos+intakeServo2Offset+intakeServoOffsetDuringGame);
                GateServo.setPosition(OpenPos);
                intakeMotor.setPower(1);
//                outtakeMotor.setPower(0.7);
//                flywheelOn(getFlywheelVel1() + 1000);
//                if (stateTimer.seconds() > ShotTime-0.2 && stateTimer.seconds() < ShotTime) {
//                    intakeMotor.setPower(1);
//                    outtakeMotor.setPower(1);
//                }
                if (stateTimer.seconds() > ShotTime) {
//                    flywheelOn(getFlywheelVel1() - 1000);
                    GateServo.setPosition(ClosePos);
//                    outtakeMotor.setPower(0);
                    stateTimer.reset();
                    shooterState = ShooterState.Done;
                }
                break;

            case Done:
                GateServo.setPosition(ClosePos);
                intakeMotor.setPower(0);
                IntakeServo.setPosition(downPos+intakeServoOffsetDuringGame);
                IntakeServo2.setPosition(downPos + intakeServo2Offset+intakeServoOffsetDuringGame);
                ball = 0;
                stateTimer.reset();
                shooterState = ShooterState.Idle;
                break;
        }
//        lastDetected = ballDetected;
    }

    public boolean isBusy() {
        return shooterState != ShooterState.Idle;
    }

    public void fireShot() {
        shotRequested = true;
    }

    public void setReturnRequested() {
        returnRequested = true;
    }

    public void fullBall() {
        stateTimer.reset();
        shooterState = ShooterState.Hold;
    }

    public void flywheelOn(double vel) {
        vel = Range.clip(vel,0,ShooterConstant.maxTicks);
        shootMotor.setVelocity(vel);
        shootMotor2.setVelocity(vel);
    }

    public double getFlywheelPower1() {
        return shootMotor.getPower();
    }

    public double getFlywheelVel1() {
        return shootMotor.getVelocity();
    }

//    public double getFlywheelPower2() {
//        return shootMotor2.getPower();
//    }
//
//    public double getFlywheelVel2() {
//        return shootMotor2.getVelocity();
//    }

    public void intakeOn() {
        intakeisOn = true;
    }

    public void intakeOff() {
        intakeisOn = false;
    }

    public void setIntakeBoost(boolean boost) {
        intakeBoost = boost;
    }

    public double getIntakePower() {
        return intakeMotor.getPower();
    }

    public void outtakeOn() {
        outtakeisOn = true;
    }

    public void outtakeOff() {
        outtakeisOn = false;
    }

    public void setHood(double pos) {
        pos = Range.clip(pos,ShooterConstant.minServoPos2,ShooterConstant.maxServoPos1);
        HoodServo.setPosition(pos);
        HoodServo2.setPosition(pos);
    }

    public double getHoodPos() {
        return HoodServo.getPosition();
    }

    public void increaseIntakeOffset() {
        intakeServoOffsetDuringGame += 0.01;
    }

    public void decreaseIntakeOffset() {
        intakeServoOffsetDuringGame -= 0.01;
    }

}
