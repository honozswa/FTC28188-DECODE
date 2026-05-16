package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstant {

    // Hood calibration
    public static double angle1 = Math.toRadians(30);
    public static double maxServoPos1 = 1;
    public static double angle2 = Math.toRadians(60);
    public static double minServoPos2 = 0;
    public static double maxAngle = Math.toRadians(60);
    public static double minAngle = Math.toRadians(30);

    public static double CloseModePos = 0.5;
    public static double MidModePos = 0.5;
    public static double FarModePos = 0.5;
    public static double closeRange = 95; // [0-95]
    public static double midRange = 120; // [95-120]

    // Gate Servo
    public static final double openPos = 0.3097;
    public static final double closePos = 0;
    public static final double shotTime = 0.8;
    public static final double lastBallTransportTime = 0.4;

    // Flywheel calibration
    public static final double FlywheelOffset = 0;
    public static final double minTicks = 800;
    public static final double maxTicks = 1800;
    public static final PIDFCoefficients shooterPIDF = new PIDFCoefficients(135, 0, 0, 14.2);
    public static double ZeroVel = 0;
    public static double CloseVel = 1100;
    public static double FarVel = 1720;
    public static double MidVel = 1250;
    public static final double VelStep = 20;

    public static double kI = 0;

    // Goal Constant (meters)
    public static double entryAngle = Math.toRadians(-25);
    static double goalHeight = 0.985;
    static double ShooterHeight = 0.279;
    public static double entryHeight = goalHeight - ShooterHeight; // meters
    public static double g = 9.81;

    // Intake
    public static double intakeServoDownPos = 0.57;
    public static double intakeServoUpPos = 0;
    public static double intakeServoPushPos = 1;


}
