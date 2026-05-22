package org.firstinspires.ftc.teamcode.mechanism;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class Util {

    public static double getFlywheelVelocityFromDistance(double distance) {
        return -0.00000662896 * Math.pow(distance, 4)
                + 0.00254963 * Math.pow(distance, 3)
                - 0.339279 * Math.pow(distance, 2)
                + 23.93036 * distance
                + 343.57827
                + ShooterConstant.AutoFlywheelOffset;
    }

    public static double getHoodPositionFromDistance(double distance) {
        return -(7.217e-9) * Math.pow(distance, 4)
                + 0.00000294991 * Math.pow(distance, 3)
                - 0.000433966 * Math.pow(distance, 2)
                + 0.0287524 * distance
                - 0.639947
                + ShooterConstant.AutoHoodOffset;
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
