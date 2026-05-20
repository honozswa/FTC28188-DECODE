package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;

public class PoseConstant {

    // BLUE POSE
    public static final Pose BLUE_GOAL = new Pose(4 , 140);

    public static final Pose BluePark = new Pose(105, 33);

    public static final Pose BlueCloseAutoStartPose = new Pose(48, 135, Math.toRadians(270));


    // RED POSE
    public static final Pose RED_GOAL = new Pose(140 , 140);
    public static final Pose RedPark = new Pose(39, 33);
    public static final Pose RedCloseAutoStartPose = new Pose(144-48, 135, Math.toRadians(-90));
    public static final Pose RedFarAutoStartPose = new Pose(96, 9, Math.toRadians(90));

    // Pose Storage
    public static Pose AutoEndPose = new Pose();
    public static boolean hasAutoPose = false;

}
