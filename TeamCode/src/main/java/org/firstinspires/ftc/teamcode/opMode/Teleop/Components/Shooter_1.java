package org.firstinspires.ftc.teamcode.opMode.Teleop.Components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter_1 {
    private DcMotorEx  flyWheel = null;
    private Servo Launcher;

    public void init(HardwareMap hwMap) {
        flyWheel = hwMap.get(DcMotorEx.class, "flyWheelR");

        Launcher = hwMap.get(Servo.class, "Launcher");

        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void shootArtifact(double shoot, double launcher) {
        flyWheel.setPower(shoot);

        Launcher.setPosition(launcher);

    }
}
