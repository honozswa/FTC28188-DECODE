package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Otos {

    SparkFunOTOS otos;

    public void init(HardwareMap hwMap) {
        otos = hwMap.get(SparkFunOTOS.class,"otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.setLinearScalar(1);
        otos.setAngularScalar(1);
        otos.calibrateImu();
        otos.resetTracking();
    }

    public double getOtosHeading() {
        return otos.getPosition().h;
    }

    public void setOtosHeading(double heading) {
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, heading));
    }

    public void resetPos() {
        otos.resetTracking();
    }

}
