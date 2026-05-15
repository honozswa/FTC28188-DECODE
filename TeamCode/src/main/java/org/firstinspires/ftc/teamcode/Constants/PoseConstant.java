package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;

public class PoseConstant {

    // BLUE POSE
    public static final Pose BLUE_GOAL = new Pose(4 , 140);

    public static final Pose BLUE_SHOOT_POSE = new Pose(48, 100, Math.toRadians(140));

    public static final Pose BlueAutoStartPose = new Pose(48, 135, Math.toRadians(270));


    // RED POSE
    public static final Pose RED_GOAL = new Pose(140 , 140);
    public static final Pose RedCloseAutoStartPose = new Pose(144-48, 135, Math.toRadians(270));

    // Pose Storage
    public static Pose AutoEndPose = new Pose();
    public static boolean hasAutoPose = false;

}
