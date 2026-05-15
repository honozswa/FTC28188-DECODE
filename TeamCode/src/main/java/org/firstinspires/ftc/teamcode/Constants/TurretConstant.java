package org.firstinspires.ftc.teamcode.Constants;

public class TurretConstant {

    public static final double angleTolerance = Math.toRadians(0.01);

    // Odo Coefficients
    public static final double OdokP = 3;
    public static final double OdokD = 0.1;
    public static final double OdokF = 0.11;

    // Cam Coefficients
    public static final double CamkP = 0.0230;
    public static final double CamkD = 0;
    public static final double CamkF = 0.12;
    public static final double CamOffset = -2.5; // (-) -> right

    // Turret Constraints
    public static final double MAX_POWER = 1;
    public static final double MAX_ANGLE = Math.toRadians(170);
    public static final double MIN_ANGLE = Math.toRadians(-20);
    public static final double TICKS_PER_DEGREE = 6.24444;
    public static final double lowPowerThreshold = 0.097;
    public static final double exponent = 0.665337;

    // Turret Angle Storage
    public static double TurretAngleOffset = 0;
    public static boolean hasTurretAngle = false;


}
