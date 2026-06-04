package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class ShooterV6Far {
    private DcMotor intakeMotor, outtakeMotor;
    private DcMotorEx shootMotor, shootMotor2;
    private Servo HoodServo, HoodServo2, GateServo, GateServo2;
    DistanceSensor distanceSens = new DistanceSensor();
    ColorSensor colorSensor = new ColorSensor();
    ColorSensor.DetectedColor detectedColor;
    ColorSensor.DetectedColor2 detectedColor2;
    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean outtakeTiming = false;
    private ElapsedTime outtakeTimer = new ElapsedTime();
    private ElapsedTime noBallTimer = new ElapsedTime();
    private boolean noBallTiming = false;
    public enum ShooterState {
        Idle,
        Intake,
        Hold,
        ThreeBall,
        Shot,
        ShotManual,
        FarShotManual,
        Done,
    }
    private ShooterState shooterState;

    // Gate
    private boolean shotRequested = false;
    private boolean returnRequested = false;
    private boolean fireManualRequested = false;
    private boolean farFireManualRequested = false;

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
        GateServo2 = hwMap.get(Servo.class,"GateServo2");
        distanceSens.init(hwMap);
        colorSensor.init(hwMap);

        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
        HoodServo2.setDirection(Servo.Direction.REVERSE);

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstant.shooterPIDF);

        setHood(ShooterConstant.minHoodPos);
        closeGate();

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateTimer.reset();
        shooterState = ShooterState.Idle;
    }

    public void update() {

        // boolean
        detectedColor = colorSensor.getDetectedBall();
        boolean lowSensorDetected = detectedColor != ColorSensor.DetectedColor.UNKNOWN;

        detectedColor2 = colorSensor.getDetectedBall2();
        boolean midSensorDetected = detectedColor2 != ColorSensor.DetectedColor2.UNKNOWN;

        boolean highSensorDetected = distanceSens.ballDetection();

        // state machine
        switch (shooterState) {
            case Idle:
                intakeMotor.setPower(0);
                outtakeMotor.setPower(0);
                closeGate();
                if (intakeisOn) {
                    shooterState = ShooterState.Intake;
                }
                if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                if (fireManualRequested) {
                    stateTimer.reset();
                    shooterState = ShooterState.ShotManual;
                }
                if (farFireManualRequested) {
                    stateTimer.reset();
                    shooterState = ShooterState.FarShotManual;
                }
                break;

            case Intake:
                intakeMotor.setPower(intakeBoost ? 1 : 0);
                outtakeMotor.setPower(intakeBoost ? 1 : 0);
                closeGate();
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
                if (fireManualRequested) {
                    stateTimer.reset();
                    shooterState = ShooterState.ShotManual;
                }
                if (farFireManualRequested) {
                    stateTimer.reset();
                    shooterState = ShooterState.FarShotManual;
                }
                break;

            case Hold:
                intakeMotor.setPower(intakeBoost ? 1 : 0);
                outtakeMotor.setPower(outtakeisOn ? 0.65 : 0);
                closeGate();
                if (!intakeisOn) {
                    shooterState = ShooterState.Idle;
                }
//                if (highSensorDetected && midSensorDetected && lowSensorDetected) {
//                    outtakeTiming = false;
//                    stateTimer.reset();
//                    shooterState = ShooterState.ThreeBall;
//                }
//                if (highSensorDetected && midSensorDetected && !lowSensorDetected) {
//                    if (!outtakeTiming) {
//                        outtakeTiming = true;
//                        outtakeTimer.reset();
//                    }
//                    if (outtakeTimer.seconds() > 0.5) {
//                        outtakeMotor.setPower(0);
//                    }
//                } else {
//                    outtakeMotor.setPower(0.65);
//                }

                if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                if (fireManualRequested) {
                    stateTimer.reset();
                    shooterState = ShooterState.ShotManual;
                }
                if (farFireManualRequested) {
                    stateTimer.reset();
                    shooterState = ShooterState.FarShotManual;
                }
                break;

            case ThreeBall:
                if (stateTimer.seconds() > 1.5) {
                    intakeMotor.setPower(0);
                    outtakeMotor.setPower(0);
                    closeGate();
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
                    if (fireManualRequested) {
                        stateTimer.reset();
                        shooterState = ShooterState.ShotManual;
                    }
                    if (farFireManualRequested) {
                        stateTimer.reset();
                        shooterState = ShooterState.FarShotManual;
                    }
                }
                if (returnRequested) {
                    returnRequested = false;
                    if (midSensorDetected || highSensorDetected) {
                        shooterState = ShooterState.Hold;
                    } else {
                        shooterState = ShooterState.Intake;
                    }
                }
//                if (!highSensorDetected || !midSensorDetected || !lowSensorDetected) {
//                    stateTimer.reset();
//                    shooterState = ShooterState.Intake;
//                }
                if (shotRequested) {
                    shotRequested = false;
                    stateTimer.reset();
                    shooterState = ShooterState.Shot;
                }
                if (fireManualRequested) {
                    stateTimer.reset();
                    shooterState = ShooterState.ShotManual;
                }
                if (farFireManualRequested) {
                    stateTimer.reset();
                    shooterState = ShooterState.FarShotManual;
                }
                break;

            case Shot:
                openGate();
                intakeMotor.setPower(0.6);
                outtakeMotor.setPower(0.6);
                boolean noBallDetected = !lowSensorDetected && !midSensorDetected && !highSensorDetected;
                if (noBallDetected) {
                    if (!noBallTiming) {
                        noBallTiming = true;
                        noBallTimer.reset();
                    }
                    if (noBallTimer.seconds() > ShooterConstant.lastBallTransportTime) {
                        noBallTiming = false;
                        stateTimer.reset();
                        shooterState = ShooterState.Done;
                    }
                } else {
                    noBallTiming = false;
                }
                break;

            case ShotManual:
                if (fireManualRequested) {
                    openGate();
                    intakeMotor.setPower(1);
                    outtakeMotor.setPower(1);
                } else {
                    shooterState = ShooterState.Done;
                }

                break;

            case FarShotManual:
                if (farFireManualRequested) {
                    openGate();
                    intakeMotor.setPower(0.6);
                    outtakeMotor.setPower(0.6);
                } else {
                    shooterState = ShooterState.Done;
                }

                break;

            case Done:
                closeGate();
                if (!intakeisOn) {
                    stateTimer.reset();
                    shooterState = ShooterState.Idle;
                } else {
                    stateTimer.reset();
                    shooterState = ShooterState.Intake;
                }
                break;
        }
    }

    public boolean isBusy() {
        return shooterState != ShooterState.Idle && shooterState != ShooterState.Intake;
    }

    public void fireShot() {
        shotRequested = true;
    }

    public void fireManualOn() {
        fireManualRequested = true;
    }

    public void fireManualOff() {
        fireManualRequested = false;
    }

    public void farFireOn() {
        farFireManualRequested = true;
    }

    public void farFireOff() {
        farFireManualRequested = false;
    }

    public void setReturnRequested() {
        returnRequested = true;
    }

    public void fullBall() {
        stateTimer.reset();
        shooterState = ShooterState.ThreeBall;
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
        double pos1 = Range.clip(pos,ShooterConstant.minHoodPos,ShooterConstant.maxHoodPos);
        double pos2 = Range.clip(pos+ShooterConstant.hoodServo2Offset, ShooterConstant.minHoodPos, ShooterConstant.maxHoodPos);
        HoodServo.setPosition(pos1);
        HoodServo2.setPosition(pos2);
    }

    public double getHoodPos() {
        return HoodServo.getPosition();
    }

    public double getGatePos() {
        return GateServo.getPosition();
    }

    public void closeGate() {
        GateServo.setPosition(ShooterConstant.closePos);
        GateServo2.setPosition(ShooterConstant.closePos2);
    }

    public void openGate() {
        GateServo.setPosition(ShooterConstant.openPos);
        GateServo2.setPosition(ShooterConstant.openPos2);
    }

    public boolean isThreeBall() {
        detectedColor = colorSensor.getDetectedBall();
        boolean lowSensorDetected = detectedColor != ColorSensor.DetectedColor.UNKNOWN;
        detectedColor2 = colorSensor.getDetectedBall2();
        boolean midSensorDetected = detectedColor2 != ColorSensor.DetectedColor2.UNKNOWN;
        boolean highSensorDetected = distanceSens.ballDetection();
        return  lowSensorDetected && midSensorDetected && highSensorDetected;
    }

    public boolean lowSensorDetected() {
        detectedColor = colorSensor.getDetectedBall();
        return detectedColor != ColorSensor.DetectedColor.UNKNOWN;
    }

    public boolean midSensorDetected() {
        detectedColor2 = colorSensor.getDetectedBall2();
        return detectedColor2 != ColorSensor.DetectedColor2.UNKNOWN;
    }

    public boolean highSensorDetected() {
        return distanceSens.ballDetection();
    }

    public ShooterState getState() {
        return shooterState;
    }

}
