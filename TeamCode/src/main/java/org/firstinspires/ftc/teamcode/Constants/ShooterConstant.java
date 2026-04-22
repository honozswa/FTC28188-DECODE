package org.firstinspires.ftc.teamcode.Constants;

public class ShooterConstant {

    // Hood calibration
    public static double angle1 = Math.toRadians(25);
    public static double servoPos1 = 0.5;
    public static double angle2 = Math.toRadians(65);
    public static double servoPos2 = 0.15;
    public static double maxAngle = Math.toRadians(65);
    public static double minAngle = Math.toRadians(25);

    // Flywheel calibration
    public static final double FlywheelOffset = 50;

    // Goal Constant (meters)
    public static double entryAngle = Math.toRadians(-25);
    static double goalHeight = 0.985;
    static double ShooterHeight = 0.305;
    public static double entryHeight = goalHeight - ShooterHeight; // meters
    public static double entryRadius = 0.127; // meters
    public static double g = 9.81;

}
