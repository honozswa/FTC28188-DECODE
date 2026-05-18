package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstant {

    /// Hood
    public static double maxHoodPos = 0.3593;
    public static double minHoodPos = 0;
    public static final double hoodServo2Offset = 0; // Servo1 is Expansion hub side, 2 is on Control hub side.
    public static final double hoodStep = 0.03;

    /// Gate
    public static final double openPos = 0.6011; // 1 - ExpansionHub , 2 - ControlHub
    public static final double closePos = 1;
    public static final double openPos2 = 0.4267;
    public static final double closePos2 = 0;

    public static final double shotTime = 0.6;
    public static final double lastBallTransportTime = 0.3;

    /// Flywheel
    public static final double FlywheelOffset = 0;
    public static final double minTicks = 800;
    public static final double maxTicks = 1600;
    public static final PIDFCoefficients shooterPIDF = new PIDFCoefficients(155, 0, 0, 17.83);
    public static double ZeroVel = 0;
    public static double CloseVel = 1200;
    public static double FarVel = 1660;
    public static double MidVel = 1420;
    public static final double VelStep = 20;

    /// Aimbot PID Coefficients
    // Odometry PID
    public static final double odokP = 1.5;
    public static final double odokD = 0.05;
    public static final double odoOffset = Math.toRadians(0); // Degrees (-) -> left

    // Limelight PID
    public static final double llkP = 0.0230;
    public static final double llkD = 0;
    public static final double llOffset = -2.5; // (-) -> right
    public static final int redTagPipeline = 4; // 2, 3, 4
    public static final int blueTagPipeline = 1; // 0, 1

    // Non-linear scaling
    public static final double lowPowerThreshold = 0.2;
    public static final double exponent = 0.6;

    // Constraint
    public static final double angleTolerance = Math.toRadians(0.01);
    public static final double maxRotatePower = 1;

}
