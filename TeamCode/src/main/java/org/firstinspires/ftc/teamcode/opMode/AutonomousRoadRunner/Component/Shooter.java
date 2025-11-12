//package org.firstinspires.ftc.teamcode.opMode.AutonomousRoadRunner.Component;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@
//public class Shooter {
//
//    public DcMotor shootWheel;
//    public Servo artifactStooper;
//    public boolean isShooting = false;
//    public static final double WHEEL_SHOOT_POWER = 0.8;
//
//    int artifact = 3;
//
//    public void init(HardwareMap HwMap) {
//        shootWheel = HwMap.get(DcMotor.class, "shootWheel");
//        artifactStooper = HwMap.get(Servo.class, "artifactStooper");
//
//        artifactStooper.setPosition(0.2);
//        shootWheel.setPower(0);
//    }
//
//    public void aShooter() {
//        isShooting = true;
//
//        artifactStooper.setPosition(0);
//        shootWheel.setPower(WHEEL_SHOOT_POWER);
//        sleep(250);
//        artifactStooper.setPosition(0.2);
//        sleep(200);
//        shootWheel.setPower(0);
//
//        sleep(1500);
//        isShooting = false;
//
//    }
//
//    public void ThreeAS() {
//        while (opModeIsActive() && artifact > 0) {
//            aShooter();
//            artifact -= 1;
//        }
//    }
//}
