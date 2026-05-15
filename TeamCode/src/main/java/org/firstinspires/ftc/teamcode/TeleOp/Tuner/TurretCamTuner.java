package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.PoseConstant;
import org.firstinspires.ftc.teamcode.Constants.TurretConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import org.firstinspires.ftc.teamcode.mechanism.Util;

@TeleOp(name = "Tuner-TurretCam", group = "Tuner")
public class TurretCamTuner extends OpMode {
    private Limelight3A limelight;
    private Turret turret = new Turret();
    private MecanumDrive drive = new MecanumDrive();
    Follower follower;
    boolean turretOn = true;

    double F = TurretConstant.CamkF;
    double P = TurretConstant.CamkP;
    double D = TurretConstant.CamkD;
    // Camera Variables
    double lastErrorCam = 0;
    double lastErrorOdo = 0;

    // Turret
    double turretPower = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private double Offset = TurretConstant.CamOffset;

    // ---------------------- driving setup --------------------------------- //
    double forward,strafe,rotate;

    // --------------------- controller based PD tuning ----------------------- //
    double[] stepSizes = {1,0.1,0.01,0.001,0.0001,0.00001};
    int stepIndex = 1;

    @Override
    public void init() {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseConstant.BlueAutoStartPose);

//        webcam.init(hardwareMap, telemetry);
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(4); // RedTag
        limelight.start();

        turret.init(hardwareMap);
        drive.init(hardwareMap);

        timer.reset();
    }

    @Override
    public void init_loop() {
        if (limelight.isConnected()) {;
            telemetry.addLine("Limelight Connected");
            telemetry.addLine("Starting...");
            telemetry.update();
            if (limelight.isRunning()) {
                telemetry.addLine("Ready to start");
                telemetry.update();
            }
        } else {
            telemetry.addLine("Camera is not Connected");
            if (!limelight.isConnected()) {
                limelight.stop();
                sleep(1000);
                limelight.start();
            }
            telemetry.update();
        }
    }

    @Override
    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();

        // Mecanum Drive Input
        forward =- gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

//        if (gamepad1.xWasPressed()) {
//            useCamera = !useCamera;
//        }

//        webcam.update();
//        AprilTagDetection id20 = webcam.getTagBySpecificId(20);
        LLResult result = limelight.getLatestResult();
        TurretAim(result);
//        if (turretOn) {
//            if (useCamera) {
//                TurretAim();
//            } else {
//                turret.setTargetAngle(curAngle);
//            }
//        }

//        if (!useCamera) {
//            drive.drive(0,0,0);
//        } else {
//            drive.drive(forward, strafe, rotate);
//        }

        if (gamepad1.leftBumperWasPressed()) {
            turret.resetEncoder();
        }

//        turret.update(follower);

        // Drive
        drive.drive(forward,strafe,rotate);

        // Tuner
//        if (gamepad1.yWasPressed()) {
//            if (curAngle == centerAngle) {
//                curAngle = offAngle;
//            } else { curAngle = centerAngle; }
//        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.aWasPressed()) {
            turretOn = !turretOn;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad2.dpadUpWasPressed()) {
            D += stepSizes[stepIndex];
        }

        if (gamepad2.dpadDownWasPressed()) {
            D -= stepSizes[stepIndex];
        }

        if (gamepad2.dpadLeftWasPressed()) {
            Offset += 0.1;
        }

        if (gamepad2.dpadRightWasPressed()) {
            Offset -= 0.1;
        }


        if (result.isValid()) {
            telemetry.addLine("Detected");
        } else {
            telemetry.addLine("No Tag Detected");
        }

        telemetry.addLine("--------------------------");
        telemetry.addLine("leftBumper for turret heading reset");
        telemetry.addData("TurretOn", turretOn);
//        telemetry.addData("Camera in use (x)", useCamera);
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)", F);
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning D","%.4f (D-Pad2 U/D)", D);
        telemetry.addData("Step Size","%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.addData("Offset (D-Pad2 L/R)", Offset);
        telemetry.addLine("--------------------------");
        telemetry.addData("tx", result.getTx());
        telemetry.addData("detected?", result.isValid());
        telemetry.addData("power", turretPower);
        telemetry.addData("TurretHeading", turret.getCurrentAngle());
        telemetry.addData("forward",follower.getPose().getX());
        telemetry.addData("strafe",follower.getPose().getY());
        telemetry.addData("Heading",Math.toDegrees(follower.getPose().getHeading()));

    }

    public void TurretAim(LLResult result) {
        if (turretOn) {
            Pose robotPose = follower.getPose();
            double dt = timer.seconds();
            timer.reset();
            if (result.isValid()) {
                // ===== CAMERA AIM =====
//                double turretAngle = Math.toRadians(turret.getCurrentAngle());
//                double txRad = Math.toRadians(result.getTx());
//                double turretTarget = Util.angleWrap(turretAngle - txRad);
//                double clippedTarget = Range.clip(turretTarget, TurretConstant.MIN_ANGLE, TurretConstant.MAX_ANGLE);
//                double error = clippedTarget - turretAngle;
                double turretAngle = turret.getCurrentAngle();
                double txRad = result.getTx();
                double error = -txRad + Offset;

                boolean atMin = turretAngle <= Math.toDegrees(TurretConstant.MIN_ANGLE);
                boolean atMax = turretAngle >= Math.toDegrees(TurretConstant.MAX_ANGLE);

                if ((atMin && error < 0) || (atMax && error > 0)) {
                    error = 0;
                }

                double pTerm = error * P;
                double dTerm = 0;
                if (dt > 0) {
                    dTerm = ((error - lastErrorCam) / dt) * D;
                }
                lastErrorCam = error;
                double FeedForward = F * follower.getAngularVelocity();
                turretPower = pTerm + dTerm - FeedForward;
                turret.setPower(Range.clip(turretPower,-TurretConstant.MAX_POWER,TurretConstant.MAX_POWER));

                telemetry.addLine("Camera in use");
                telemetry.addData("turretTicks", turret.getTicks());
                telemetry.addData("turretAngle", Math.toDegrees(turretAngle));
                telemetry.addData("tx", Math.toDegrees(txRad));
//                telemetry.addData("turretTarget", Math.toDegrees(turretTarget));
//                telemetry.addData("clippedTarget", Math.toDegrees(clippedTarget));
                telemetry.addData("error", error);
                telemetry.addData("turretPower", turretPower);
                telemetry.addLine("");


            } else {
                turret.setPower(0);
            }

        } else {
            turret.setPower(gamepad2.left_stick_x * 0.5);
        }
    }

}
