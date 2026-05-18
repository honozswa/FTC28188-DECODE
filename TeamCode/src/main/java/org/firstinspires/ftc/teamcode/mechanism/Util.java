package org.firstinspires.ftc.teamcode.mechanism;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class Util {

    public static double getFlywheelVelocityFromDistance(double distance) {
        return - 0.00000599859 * Math.pow(distance, 4)
                + 0.00259486 * Math.pow(distance, 3)
                - 0.403602 * Math.pow(distance, 2)
                + 31.91837 * distance
                + 54.40743
                + ShooterConstant.AutoFlywheelOffset;
    }

    public static double getHoodPositionFromDistance(double distance) {
        return - (1.05483e-8) * Math.pow(distance, 4)
                + 0.00000516111 * Math.pow(distance, 3)
                - 0.000939197 * Math.pow(distance, 2)
                + 0.074721 * distance
                - 1.91019
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
