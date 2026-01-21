package org.firstinspires.ftc.teamcode.opMode.Teleop.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearMotorLift {
    private DcMotor leftMotor,rightMotor;
    private final boolean LRMotor = false;

    private static final int HIGH_MODE = 1500;
    private static final int GROUND_MODE = 0;
    private static final int MEDIUM_MODE = 1000;

    private void parkingAdjust(int TargetPositon) {
        rightMotor.setTargetPosition(TargetPositon);
        leftMotor.setTargetPosition(TargetPositon);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.6);
        rightMotor.setPower(0.6);
    }
    public void init(HardwareMap HwMap){
        leftMotor = HwMap.get(DcMotor.class, "left_motor_Parking");
        rightMotor = HwMap.get(DcMotor.class, "right_motor_Parking");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void parking() {
        if (gamepad2.right_stick_y > 0) {
            leftMotor.setPower(0.6);
            rightMotor.setPower(0.6);

        }else if(gamepad2.right_stick_y < 0) {
            leftMotor.setPower(-0.6);
            rightMotor.setPower(-0.6);
        }else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        if (gamepad2.dpad_up) {
            parkingAdjust(HIGH_MODE);
        }else if (gamepad2.dpad_down) {
            parkingAdjust(GROUND_MODE);
        }else if (gamepad2.dpad_left) {
            parkingAdjust(MEDIUM_MODE);
        }
    }
}
