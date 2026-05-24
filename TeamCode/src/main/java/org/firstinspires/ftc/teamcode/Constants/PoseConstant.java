package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;

public class PoseConstant {

    // BLUE POSE
    public static final Pose BLUE_GOAL = new Pose(4 , 140);
    public static final Pose BluePark = new Pose(105, 33, Math.toRadians(0));
    public static final Pose BlueGate = new Pose(16,59,Math.toRadians(150));
    public static final Pose BlueCloseAutoStartPose = new Pose(48, 135, Math.toRadians(270));
    public static final Pose BlueFarAutoStartPose = new Pose(48, 9, Math.toRadians(90));
    public static final Pose BlueResetPose = new Pose(144-129,82,Math.toRadians(180));
    public static final Pose BlueFarResetPose = new Pose(135,8,Math.toRadians(0));


    // RED POSE
    public static final Pose RED_GOAL = new Pose(140 , 140);
    public static final Pose RedPark = new Pose(39, 33, Math.toRadians(180));
    public static final Pose RedGate = new Pose(128,57,Math.toRadians(30));
    public static final Pose RedCloseAutoStartPose = new Pose(96, 135, Math.toRadians(-90));
    public static final Pose RedFarAutoStartPose = new Pose(96, 9, Math.toRadians(90));
    public static final Pose RedResetPose = new Pose(129,82,Math.toRadians(0));
    public static final Pose RedFarResetPose = new Pose(9,8,Math.toRadians(180));

    // Pose Storage
    public static Pose AutoEndPose = new Pose();
    public static boolean hasAutoPose = false;

}
