package org.firstinspires.ftc.teamcode.opMode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opMode.Teleop.Components.LinearMotorLift;
import org.firstinspires.ftc.teamcode.opMode.Teleop.Components.Movement;
import org.firstinspires.ftc.teamcode.opMode.Teleop.Components.Shooter;
import org.firstinspires.ftc.teamcode.opMode.Teleop.Components.Shooter_1;
import org.firstinspires.ftc.teamcode.opMode.aprilTag.apritagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Teleop", group = "Go2Steam")
public class mainTeleop extends OpMode {
    Movement driveBaseMovement = new Movement();
    Shooter shoot = new Shooter();
    Shooter_1 shooter = new Shooter_1();
    apritagDetection apriltagDetection = new apritagDetection();

    LinearMotorLift parkingMachine = new LinearMotorLift();


    @Override
    public void init() {
        driveBaseMovement.init(hardwareMap);
        shoot.init(hardwareMap);
        shooter.init(hardwareMap);
        parkingMachine.init(hardwareMap);
        apriltagDetection.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        apriltagDetection.update();
        AprilTagDetection id20 = apriltagDetection.getTagBySpesificId(20);
        AprilTagDetection id21 = apriltagDetection.getTagBySpesificId(21);
        AprilTagDetection id22 = apriltagDetection.getTagBySpesificId(22);
        telemetry.addData("ID 20", id20.toString());
        telemetry.addData("ID 21", id21.toString());
        telemetry.addData("ID 22", id22.toString());

        double forward = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;
        driveBaseMovement.drive(forward,rotate);

        shooter.shootArtifact();
        shoot.shootMechanism();

        parkingMachine.parking();
    }
}
