package org.firstinspires.ftc.teamcode.mechanism;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class Util {

    public static double getFlywheelVelocityFromDistance(double distance) {
        return (5.47483 * distance) + 689.44141 + ShooterConstant.AutoFlywheelOffset;
    }

    public static double getHoodPositionFromDistance(double distance) {
        return 0.126656 / (1 + Math.exp(-(0.121999 * distance - 7.55691))) + ShooterConstant.AutoHoodOffset;
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
