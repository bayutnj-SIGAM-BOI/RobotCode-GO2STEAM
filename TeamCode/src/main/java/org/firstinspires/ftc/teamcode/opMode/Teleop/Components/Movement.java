package org.firstinspires.ftc.teamcode.opMode.Teleop.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Movement {
    private DcMotor leftDrive,rightDrive;
;
    public void init(HardwareMap HwMap) {
        leftDrive = HwMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = HwMap.get(DcMotor.class, "right_motor_tDrive");


        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double forward, double rotate) {
        double leftPower = forward + rotate;
        double rightPower = forward - rotate;

        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}