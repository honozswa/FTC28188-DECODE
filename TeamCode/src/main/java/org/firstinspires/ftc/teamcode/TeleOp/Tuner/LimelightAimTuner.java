package org.firstinspires.ftc.teamcode.TeleOp.Tuner;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstant;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;

@TeleOp(name = "Tuner-LimelightAim", group = "Tuner")
public class LimelightAimTuner extends OpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private Limelight3A limelight;

    // ------------------------- PD Controller ------------------------ //
    double kP = ShooterConstant.llkP;
    double kD = ShooterConstant.llkD;
    double Offset = ShooterConstant.llOffset;
    double lastErrorLL = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // ---------------------- driving setup --------------------------------- //
    double forward,strafe,rotate;

    // --------------------- controller based PD tuning ----------------------- //
    double[] stepSizes = {1,0.1,0.01,0.001,0.0001};
    int stepIndex = 2;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(ShooterConstant.redTagPipeline);
        limelight.start();

        drive.init(hardwareMap);

        if (limelight.isConnected() && limelight.isRunning()) {;
            telemetry.addLine("Limelight Connected");
            telemetry.addLine("Ready to start");
        } else {
            telemetry.addLine("Limelight is not Connected");
            telemetry.addLine("Please reconnect the Limelight cable");
        }
    }

    @Override
    public void loop() {
        // Mecanum Drive Input
        forward =- gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // Auto Align Logic
        Aimbot(limelight.getLatestResult());

        // Drive
        drive.drive(forward,strafe,rotate);

        // Tuner
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadDownWasPressed()) {
            kP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            kP += stepSizes[stepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            kD -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad2.dpadUpWasPressed()) {
            Offset += stepSizes[stepIndex];
        }
        if (gamepad2.dpadDownWasPressed()) {
            Offset -= stepSizes[stepIndex];
        }

        // Telemetry
        if (limelight.getLatestResult().isValid()) {
            if (gamepad1.left_trigger > 0.5) {
                telemetry.addLine("Auto Align");
            }
        } else {
            telemetry.addLine("Manual Rotate Mode");
        }
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)", kP);
        telemetry.addData("Tuning D","%.4f (D-Pad L/R)", kD);
        telemetry.addData("Tuning Offset","%.4f (D-Pad2 U/D)", Offset);
        telemetry.addData("Step Size","%.4f (B Button)", stepSizes[stepIndex]);

    }

    public void Aimbot(LLResult result) {
        if (gamepad1.left_trigger > 0.5) {
            double dt = timer.seconds();
            timer.reset();
            if (result.isValid()) {
                // ===== CAMERA AIM =====
                double error = result.getTx() + Offset;
                double pTerm = error * kP;
                double dTerm = 0;
                if (dt > 0) { dTerm = ((error - lastErrorLL) / dt) * kD;}
                double rawRotate = Range.clip(pTerm + dTerm, -ShooterConstant.maxRotatePower, ShooterConstant.maxRotatePower);
                if (Math.abs(rawRotate) < ShooterConstant.lowPowerThreshold) {
                    rotate = Math.signum(rawRotate) * Math.pow(Math.abs(rawRotate), ShooterConstant.exponent);
                } else {
                    rotate = rawRotate;
                }
                lastErrorLL = error;
                telemetry.addLine("Camera in use");
                telemetry.addData("error", error);
                telemetry.addData("rotate", rotate);
                telemetry.addLine("");
            }
        }
    }

}
