package org.firstinspires.ftc.teamcode.opMode.Teleop.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Tuning Shooter")
public class ShooterTunning extends OpMode {
    private DcMotorEx flyWheel;
    double P = 0;
    double F = 0;

    double HighVelocity = 2000;
    double LowVelocity = 1000;
    double curVelocityTarget = HighVelocity;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (curVelocityTarget== HighVelocity) {
                curVelocityTarget = LowVelocity;
            } else {
                curVelocityTarget = HighVelocity;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flyWheel.setVelocity(curVelocityTarget);
        double curVelocity = flyWheel.getVelocity();
        double error = curVelocityTarget - curVelocity;

        telemetry.addData("error", "%.2f",error);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Tuning P", "%.4f (D-pad U/D", P);
        telemetry.addData("Tuning F" , "%.4f (D-pad L/R", F);
        telemetry.addData("Step Sizes", "%.4f (B-Button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}
