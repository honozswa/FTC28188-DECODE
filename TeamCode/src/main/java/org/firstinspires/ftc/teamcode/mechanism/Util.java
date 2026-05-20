package org.firstinspires.ftc.teamcode.mechanism;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;

public class Util {

    public static double getFlywheelVelocityFromDistance(double distance) {
        return 0.00000552499 * Math.pow(distance, 4)
                - 0.00221618 * Math.pow(distance, 3)
                + 0.329379 * Math.pow(distance, 2)
                - 15.83828 * distance
                + 1133.39761
                + ShooterConstant.AutoFlywheelOffset;
    }

    public static double getHoodPositionFromDistance(double distance) {
        return (2.02347e-9) * Math.pow(distance, 4)
                - (8.06964e-7) * Math.pow(distance, 3)
                + 0.0000942421 * Math.pow(distance, 2)
                - 0.00121067 * distance
                - 0.0523633
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
