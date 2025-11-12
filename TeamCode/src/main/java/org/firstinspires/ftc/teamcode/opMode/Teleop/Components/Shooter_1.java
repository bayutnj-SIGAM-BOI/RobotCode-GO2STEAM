package org.firstinspires.ftc.teamcode.opMode.Teleop.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter_1 {
    private DcMotor flyWheelL, flyWheelR;
    private Servo artifactStooper;

    public void init(HardwareMap hwMap) {
        flyWheelL = hwMap.get(DcMotor.class, "flyWheelL");
        flyWheelR = hwMap.get(DcMotor.class, "flyWheelR");
        artifactStooper = hwMap.get(Servo.class, "artifactStooper");

        flyWheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelR.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        artifactStooper.setPosition(0.2);
    }

    private void setupDcMotor(double lP,double rP) {
        flyWheelR.setPower(rP);
        flyWheelL.setPower(lP);
    }

    public void shootArtifact() {
        if (gamepad1.a | gamepad2.a) {
            artifactStooper.setPosition(0);
        } else if (gamepad1.y && gamepad2.y) {
            artifactStooper.setPosition(0.2);
        }

        if (gamepad1.b | gamepad2.b) {
            setupDcMotor(0.8, 0.8);
        } else if (gamepad1.x && gamepad2.x) {
            setupDcMotor(-0.8, -.8);
        }
    }
}
