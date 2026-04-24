package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstant {

    // Hood calibration
    public static double angle1 = Math.toRadians(30);
    public static double maxServoPos1 = 0.4;
    public static double angle2 = Math.toRadians(60);
    public static double minServoPos2 = 0;
    public static double maxAngle = Math.toRadians(60);
    public static double minAngle = Math.toRadians(30);

    public static double CloseModePos = 0;
    public static double MidModePos = 0.14;
    public static double FarModePos = 0.29;
    public static double closeRange = 95; // [0-95]
    public static double midRange = 120; // [95-120]

    // Gate Servo
    public static final double openPos = 0;
    public static final double closePos = 0.2;

    // Flywheel calibration
    public static final double FlywheelOffset = 50;
    public static final double minTicks = 900;
    public static final double maxTicks = 1700;
    public static final PIDFCoefficients shooterPIDF = new PIDFCoefficients(150, 0, 0.01, 17.0390);

    // Goal Constant (meters)
    public static double entryAngle = Math.toRadians(-25);
    static double goalHeight = 0.985;
    static double ShooterHeight = 0.279;
    public static double entryHeight = goalHeight - ShooterHeight; // meters
    public static double g = 9.81;

}
