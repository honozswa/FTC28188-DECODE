package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    private DcMotorEx shootMotor, shootMotor2, gateMotor;
    private DcMotor intakeMotor;
    private Servo HoodServo, HoodServo2, GateServo;
    private ElapsedTime stateTimer = new ElapsedTime();
    private enum FlywheelState {
        Idle,
        FlywheelOn,
        OuttakeOn,
        Shot,
        OuttakeOff,
        FlywheelOff,
    }
    private FlywheelState flywheelState;

    // Gate
    private double ClosePos = 0.45;
    private double OpenPos = 0.0;
    private double ShotTime = 1;

    // Hood
    double HoodPosition1 = 0.31;
    double HoodPosition2 = 1 - HoodPosition1;

    public void init(HardwareMap hwMap) {
        shootMotor = hwMap.get(DcMotorEx.class, "shootMotor");
        shootMotor2 = hwMap.get(DcMotorEx.class, "shootMotor2");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        gateMotor = hwMap.get(DcMotorEx.class, "gateMotor");
        HoodServo = hwMap.get(Servo.class, "HoodServo");
        HoodServo2 = hwMap.get(Servo.class, "HoodServo2");
        GateServo = hwMap.get(Servo.class, "GateServo");

        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        gateMotor.setDirection(DcMotor.Direction.REVERSE);

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(150 , 0, 0, 17.0390);
        shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        HoodServo.setPosition(HoodPosition1);
        HoodServo2.setPosition(HoodPosition2);
        GateServo.setPosition(ClosePos);

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheelState = FlywheelState.Idle;
    }

    public void update() {
        switch (flywheelState) {
            case FlywheelOn:
                flywheelState = FlywheelState.OuttakeOn;
                break;

            case OuttakeOn:
                gateMotor.setPower(1);
                stateTimer.reset();
                flywheelState = FlywheelState.Shot;
                break;

            case Shot:
                GateServo.setPosition(OpenPos);
                if (stateTimer.seconds() > ShotTime) {
                    GateServo.setPosition(ClosePos);
                    stateTimer.reset();
                    flywheelState = FlywheelState.OuttakeOff;
                }
                break;

            case OuttakeOff:
                gateMotor.setPower(0);
                stateTimer.reset();
                flywheelState = FlywheelState.FlywheelOff;
                break;

            case FlywheelOff:
                flywheelState = FlywheelState.Idle;
                break;
        }
    }

    public void fireShot() {
        if (flywheelState == FlywheelState.Idle) {
            stateTimer.reset();
            flywheelState = FlywheelState.FlywheelOn;
        }
    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.Idle;
    }

    public void intakeOn(double power) {
        intakeMotor.setPower(power);
    }

    public void flywheelOn(double vel) {
        shootMotor.setVelocity(vel);
        shootMotor2.setVelocity(vel);
    }

}
