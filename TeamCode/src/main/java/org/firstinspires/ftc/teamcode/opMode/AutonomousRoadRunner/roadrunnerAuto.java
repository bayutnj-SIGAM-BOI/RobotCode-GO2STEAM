package org.firstinspires.ftc.teamcode.opMode.AutonomousRoadRunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous(name = "Auto", group = "Go2steam")
public class roadrunnerAuto extends LinearOpMode {

    Servo artifactStop;
    DcMotor leftShooter, rightShooter;

    public class openServo implements InstantFunction {
        double targetPosition;

        public openServo(double targetPosition) {
            this.targetPosition = targetPosition;
        }
        @Override
        public void run() {
            artifactStop.setPosition(targetPosition);
        }
    }
    public class ShootMotors implements InstantFunction {
        double RPMMotor;
        public ShootMotors(double RPMMotor) {
            this.RPMMotor = RPMMotor;
        }

        @Override
        public void run() {
            leftShooter.setPower(RPMMotor);
            rightShooter.setPower(RPMMotor);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        artifactStop = hardwareMap.get(Servo.class, "artifactStop");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        artifactStop.setPosition(0.2);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        Pose2d beginPose = new Pose2d(new Vector2d(47,0), Math.toRadians(0));

        TankDrive drive = new TankDrive(hardwareMap, beginPose);

        telemetry.addLine("========= Autonomous Mode =========");
        telemetry.addData("Start Pose", "(%.1f, %.1f, %.1f°)", beginPose.position.x,beginPose.position.y,Math.toDegrees(beginPose.heading.toDouble()));
        telemetry.update();

        Action path = drive.actionBuilder(beginPose)
                .lineToX(-16)
                .turnTo(Math.toRadians(-130))
                .waitSeconds(1)
                .stopAndAdd(new openServo(0.0))
                .stopAndAdd(new ShootMotors(0.8))
                .splineTo(new Vector2d(56,56), Math.toRadians(0))
                .stopAndAdd(new openServo(0.2))
                .stopAndAdd(new ShootMotors(0.0))
                .waitSeconds(5)
                .splineTo(new Vector2d(-16,0), Math.toRadians(0))
                .turnTo(Math.toRadians(-120))
                .stopAndAdd(new openServo(0))
                .stopAndAdd(new ShootMotors(0.8))
                .waitSeconds(1.5)
                .stopAndAdd(new openServo(0.2))
                .stopAndAdd(new ShootMotors(0.0))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        telemetry.addLine("Starting path");

        Actions.runBlocking(new SequentialAction(path));
    }
}