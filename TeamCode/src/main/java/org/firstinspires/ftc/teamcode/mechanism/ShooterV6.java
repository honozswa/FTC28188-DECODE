package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class ShooterV6 {
    private DcMotor intakeMotor, outtakeMotor;
    private DcMotorEx shootMotor, shootMotor2;
    private Servo HoodServo, HoodServo2, GateServo;
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
    private boolean shotRequested = false;
    private boolean returnRequested = false;

    // Hood
    double HoodPos = ShooterConstant.minHoodPos;

    // Intake
    private boolean intakeisOn = false;
    private boolean intakeBoost = false;
    private boolean outtakeisOn = false;

    public void init(HardwareMap hwMap) {
        shootMotor = hwMap.get(DcMotorEx.class, "shootMotor");
        shootMotor2 = hwMap.get(DcMotorEx.class, "shootMotor2");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        outtakeMotor = hwMap.get(DcMotor.class, "gateMotor");
        HoodServo = hwMap.get(Servo.class, "HoodServo");
        HoodServo2 = hwMap.get(Servo.class, "HoodServo2");
        GateServo = hwMap.get(Servo.class, "GateServo");
        distanceSens.init(hwMap);
        colorSensor.init(hwMap);

        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
        HoodServo2.setDirection(Servo.Direction.REVERSE);

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);

        HoodServo.setPosition(HoodPos);
        HoodServo2.setPosition(HoodPos);
        GateServo.setPosition(ClosePos);

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateTimer.reset();
        shooterState = ShooterState.Idle;
    }

    public void update() {
        detectedColor = colorSensor.getDetectedBall();
        boolean lowSensorDetected = detectedColor != ColorSensor.DetectedColor.UNKNOWN;

        detectedColor2 = colorSensor.getDetectedBall2();
        boolean midSensorDetected = detectedColor2 != ColorSensor.DetectedColor2.UNKNOWN;

        boolean highSensorDetected = distanceSens.ballDetection();

        switch (shooterState) {
            case Idle:
                intakeMotor.setPower(0);
                outtakeMotor.setPower(0);
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
                intakeMotor.setPower(1);
                outtakeMotor.setPower(1);
                GateServo.setPosition(ClosePos);
                if (!intakeisOn) {
                    shooterState = ShooterState.Idle;
                }
                if (midSensorDetected || highSensorDetected) {
                    shooterState = ShooterState.Hold;
                }
                if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                break;

            case Hold:
                intakeMotor.setPower(1);
                GateServo.setPosition(ClosePos);
                if (!intakeisOn) {
                    shooterState = ShooterState.Idle;
                }
                if (highSensorDetected && midSensorDetected && lowSensorDetected) {
                    stateTimer.reset();
                    shooterState = ShooterState.ThreeBall;
                }
                else if (lowSensorDetected) {
                    outtakeMotor.setPower(1);
                }
                else if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                else {
                    outtakeMotor.setPower(0);
                }
                break;

            case ThreeBall:
                intakeMotor.setPower(0);
                outtakeMotor.setPower(0);
                GateServo.setPosition(ClosePos);
                if (returnRequested) {
                    returnRequested = false;
                    if (midSensorDetected || highSensorDetected) {
                        shooterState = ShooterState.Hold;
                    } else {
                        shooterState = ShooterState.Intake;
                    }
                }
                if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                break;

            case Shot:
                GateServo.setPosition(OpenPos);
                intakeMotor.setPower(1);
                outtakeMotor.setPower(1);
                if (!highSensorDetected && !midSensorDetected && !lowSensorDetected) {
                    stateTimer.reset();
                    shooterState = ShooterState.Done;
                }
                break;

            case Done:
                if (stateTimer.seconds() > ShooterConstant.lastBallTransportTime) {
                    if (!intakeisOn) {
                        stateTimer.reset();
                        shooterState = ShooterState.Idle;
                    } else {
                        stateTimer.reset();
                        shooterState = ShooterState.Intake;
                    }
                }
                break;
        }
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

    public double getFlywheelPower2() {
        return shootMotor2.getPower();
    }

    public double getFlywheelVel2() {
        return shootMotor2.getVelocity();
    }

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
        pos = Range.clip(pos,ShooterConstant.minHoodPos,ShooterConstant.maxHoodPos);
        HoodServo.setPosition(pos);
        HoodServo2.setPosition(pos);
    }

    public double getHoodPos() {
        return HoodServo.getPosition();
    }

    public double getGatePos() {
        return GateServo.getPosition();
    }



}
