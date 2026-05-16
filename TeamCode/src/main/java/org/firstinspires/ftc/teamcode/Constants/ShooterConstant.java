package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstant {

    /// Hood
    public static double maxHoodPos = 1;
    public static double minHoodPos = 0;

    /// Gate
    public static final double openPos = 0.3097;
    public static final double closePos = 0;
    public static final double shotTime = 0.6;
    public static final double lastBallTransportTime = 0.3;

    /// Flywheel
    public static final double FlywheelOffset = 0;
    public static final double minTicks = 800;
    public static final double maxTicks = 1800;
    public static final PIDFCoefficients shooterPIDF = new PIDFCoefficients(135, 0, 0, 14.2);
    public static double ZeroVel = 0;
    public static double CloseVel = 1200;
    public static double FarVel = 1660;
    public static double MidVel = 1420;
    public static final double VelStep = 20;

    /// Aimbot PID Coefficients
    // Odometry PID
    public static final double odokP = 1;
    public static final double odokD = 0.05;
    public static final double odoOffset = Math.toRadians(-2); // Degrees (-) -> left

    // Limelight PID
    public static final double llkP = 0.0230;
    public static final double llkD = 0;
    public static final double llOffset = -2.5; // (-) -> right
    public static final int redTagPipeline = 4; // 2, 3, 4
    public static final int blueTagPipeline = 1; // 0, 1

    // Non-linear scaling
    public static final double lowPowerThreshold = 0.097;
    public static final double exponent = 0.665337;

    // Constraint
    public static final double angleTolerance = Math.toRadians(0.01);
    public static final double maxRotatePower = 1;

}
