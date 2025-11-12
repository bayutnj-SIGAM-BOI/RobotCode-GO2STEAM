package org.firstinspires.ftc.teamcode.opMode.AutonomousEncoder.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootArtifactCode {
    private DcMotor shootWheel;
    private Servo artifactStooper;

    private boolean isShooting = false;
    private final ElapsedTime shootingTime = new ElapsedTime();


    enum State {
        SERVO_SHOOT,
        SERVO_STOP,
        SHOOT_STOP
    }

    State state = State.SERVO_SHOOT;

    public void init(HardwareMap HwMap) {
        shootWheel = HwMap.get(DcMotor.class, "shootWheel");
        artifactStooper = HwMap.get(Servo.class, "artifactStooper");
        artifactStooper.setPosition(0.2);

        state = State.SERVO_SHOOT;
    }

    public void shootMechanism() {
        if (!isShooting) {
            isShooting = true;
            state = State.SERVO_SHOOT;
        }
            switch (state) {
                case SERVO_SHOOT:
                    artifactStooper.setPosition(0);
                    shootWheel.setPower(0.8);
                    if (shootingTime.milliseconds() > 250) {
                        state = State.SERVO_STOP;
                    }
                    break;
                case SERVO_STOP:
                    artifactStooper.setPosition(0.2);
                    if (shootingTime.milliseconds() > 200) {
                        state = State.SHOOT_STOP;
                    }
                    break;
                case SHOOT_STOP:
                    shootWheel.setPower(0);
                    if (shootingTime.milliseconds() > 1500) {
                        isShooting = false;
                    }
                    break;
            }
        }
    }
