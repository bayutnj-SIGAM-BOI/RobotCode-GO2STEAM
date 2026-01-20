package org.firstinspires.ftc.teamcode.opMode.AutonomousRoadRunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import  com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TankDrive;
@Autonomous(name = "AutoRoadRunner", group = "Go2steam")
public class roadrunnerAuto extends LinearOpMode {

    Servo artifactStop;
    DcMotor leftShooter, rightShooter;



    @Override
    public void runOpMode() throws InterruptedException {
        artifactStop = hardwareMap.get(Servo.class, "artifactStooper");
        leftShooter = hardwareMap.get(DcMotor.class, "flyWheelL");
        rightShooter = hardwareMap.get(DcMotor.class, "flyWheelR");

        artifactStop.setPosition(0.2);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d beginPose = new Pose2d(new Vector2d(47,0), Math.toRadians(0));

        TankDrive drive = new TankDrive(hardwareMap, beginPose);

        telemetry.addLine("========= Autonomous Mode =========");
        telemetry.addData("Start Pose", "(%.1f, %.1f, %.1f°)", beginPose.position.x,beginPose.position.y,Math.toDegrees(beginPose.heading.toDouble()));
        telemetry.update();

        Action path = drive.actionBuilder(beginPose)
                .lineToX(10)
                .build();

        waitForStart();
        if (isStopRequested()) return;
        telemetry.addLine("Starting path");

        Actions.runBlocking(new SequentialAction(path));
    }
}