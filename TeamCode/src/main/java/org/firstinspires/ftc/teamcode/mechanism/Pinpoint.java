package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Pinpoint {
    GoBildaPinpointDriver odo;

    public void init(HardwareMap hwmap) {
        odo = hwmap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(1.65930,-5.28631, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }

    public void update() {
        odo.update();
    }

}
