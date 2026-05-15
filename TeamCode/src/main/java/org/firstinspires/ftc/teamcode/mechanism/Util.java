package org.firstinspires.ftc.teamcode.mechanism;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class Util {

    public static double getHoodServoPosFromAngle(double angle) {
        return ShooterConstant.maxServoPos1 + (ShooterConstant.minServoPos2 - ShooterConstant.maxServoPos1) * (angle - ShooterConstant.angle1) / (ShooterConstant.angle2 - ShooterConstant.angle1);
    }

    public static double getFlywheelVelocityFromV0(double v0) {
        return 339.81638 * v0 - 624.54735 + ShooterConstant.FlywheelOffset;
    }

    public static double getFlywheelVelocityFromDistance(double distance) {
        return 0.00000502676 * Math.pow(distance, 4)
                - 0.00241126 * Math.pow(distance, 3)
                + 0.412176 * Math.pow(distance, 2)
                - 23.78612 * distance
                + 1495.57704;
    }

    public static double getFlywheelVelocityFromDistanceClose(double distance) {
        return 776.27374 * Math.pow(1.00522, distance) + ShooterConstant.FlywheelOffset;
    }

    public static double getFlywheelVelocityFromDistanceMid(double distance) {
        return 776.27374 * Math.pow(1.00522, distance) + ShooterConstant.FlywheelOffset;
    }

    public static double getFlywheelVelocityFromDistanceFar(double distance) {
        return 776.27374 * Math.pow(1.00522, distance) + ShooterConstant.FlywheelOffset;
    }

    public static double getHoodPositionFromDistance(double distance) {
        return (8.07094e-9) * Math.pow(distance, 4)
                - 0.00000304562 * Math.pow(distance, 3)
                + 0.000379248 * Math.pow(distance, 2)
                - 0.0130841 * distance
                + 0.0724324;
    }

    public static double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

}
