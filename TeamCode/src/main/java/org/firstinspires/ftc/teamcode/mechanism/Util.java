package org.firstinspires.ftc.teamcode.mechanism;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class Util {

    public static double getHoodServoPosFromAngle(double angle) {
        return ShooterConstant.servoPos1 + (ShooterConstant.servoPos2 - ShooterConstant.servoPos1) * (angle - ShooterConstant.angle1) / (ShooterConstant.angle2 - ShooterConstant.angle1);
    }

    public static double getFlywheelVelocityFromV0(double v0) {
        return 339.81638 * v0 - 624.54735 + ShooterConstant.FlywheelOffset;
    }

}
