package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class Shooter {
    private DcMotorEx shootMotor, shootMotor2;
    private DcMotor intakeMotor;
    private Servo HoodServo, HoodServo2, GateServo;
    private ElapsedTime stateTimer = new ElapsedTime();
    private enum FlywheelState {
        Idle,
        FlywheelOn,
        IntakeSpeedUp,
        Shot,
        IntakeSpeedDown,
        FlywheelOff,
    }
    private FlywheelState flywheelState;

    // Gate
    private double ClosePos = ShooterConstant.closePos;
    private double OpenPos = ShooterConstant.openPos;
    private double ShotTime = 1;

    // Hood
    double HoodPosition1 = ShooterConstant.minServoPos2;
    double HoodPosition2 = 1 - HoodPosition1;

    public void init(HardwareMap hwMap) {
        shootMotor = hwMap.get(DcMotorEx.class, "shootMotor");
        shootMotor2 = hwMap.get(DcMotorEx.class, "shootMotor2");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        HoodServo = hwMap.get(Servo.class, "HoodServo");
        HoodServo2 = hwMap.get(Servo.class, "HoodServo2");
        GateServo = hwMap.get(Servo.class, "GateServo");

        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

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

        flywheelState = FlywheelState.Idle;
    }

    public void update() {
        switch (flywheelState) {
            case FlywheelOn:
                flywheelState = FlywheelState.IntakeSpeedUp;
                break;

            case IntakeSpeedUp:
                intakeMotor.setPower(1);
                stateTimer.reset();
                flywheelState = FlywheelState.Shot;
                break;

            case Shot:
                GateServo.setPosition(OpenPos);
                if (stateTimer.seconds() > ShotTime) {
                    GateServo.setPosition(ClosePos);
                    stateTimer.reset();
                    flywheelState = FlywheelState.IntakeSpeedDown;
                }
                break;

            case IntakeSpeedDown:
                intakeMotor.setPower(0.8);;
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
