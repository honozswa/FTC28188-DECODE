package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.TurretConstant;

public class Turret {

    private DcMotorEx turretMotor;
    private double MAX_POWER = TurretConstant.MAX_POWER;
    private double TICKS_PER_DEGREE = TurretConstant.TICKS_PER_DEGREE;
    private double targetAngle = 0;
    private double currentTurretAngle = 0;
    private double angleOffset = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        timer.reset();
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        turretMotor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    public boolean isAligned(double robotHeadingDeg) {
        double turretTarget = Util.angleWrap(targetAngle - robotHeadingDeg);
        double error = Util.angleWrap(turretTarget - currentTurretAngle);
        return Math.abs(error) < 2.0;
    }

    public double getCurrentAngle() {
        double ticks = turretMotor.getCurrentPosition();
        currentTurretAngle = (ticks / TICKS_PER_DEGREE) + angleOffset;
        return currentTurretAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void resetTimer() {
        timer.reset();
    }

    public double getTicks() {
        return turretMotor.getCurrentPosition();
    }

    public void setAngleOffset(double offset) {
        angleOffset = offset;
    }

}