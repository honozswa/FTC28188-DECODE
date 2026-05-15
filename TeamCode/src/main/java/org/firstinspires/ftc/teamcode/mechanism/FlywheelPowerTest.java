package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class FlywheelPowerTest {

    private DcMotorEx motor;

    private ElapsedTime timer = new ElapsedTime();

    private double lastPos;

    private double velocity;

    private double targetVelocity = 1500;

    private double power = 0;

    private double kI = ShooterConstant.kI;
    private double tolerance = 100;

    public void init(HardwareMap hardwareMap) {

        motor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lastPos = motor.getCurrentPosition();
        timer.reset();
    }

    public void update() {

        // ===== DT =====
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) return;

        // ===== VELOCITY =====
        velocity = motor.getVelocity();

        // ===== ERROR =====
        double error = targetVelocity - velocity;

        // ===== UPDATE POWER =====
        power += kI * error * dt;

        // ===== CLAMP =====
        power = Range.clip(power,0,1);

        // ===== APPLY =====
        motor.setPower(power);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void stop() {
        targetVelocity = 0;
        power = 0;
        motor.setPower(0);
    }

    public boolean atTarget() {
        return Math.abs(targetVelocity - motor.getVelocity()) < tolerance;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getPower() {
        return power;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double i) {
        this.kI = i;
    }
}
