package org.firstinspires.ftc.teamcode.opMode.Teleop;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "Teleop", group = "Go2Steam")
public class mainTeleop extends OpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx  flyWheel = null;
    private Servo Launcher;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    boolean autoAlign = false;
    double TARGET_DISTANCE = 125.0;
    static final double kP_Drive = 0.04;
    static final double kP_Bearing = 0.03;
    static final double kP_Yaw = 0.02;
    double ShooterVel = 1850;
    boolean lastA = false;
    boolean lastY = false;
    boolean ShooterOn = false;
    boolean LauncherLock = true;

    @Override
    public void init() {
//        ================ Hardware initialize ================
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_Drive");

        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheelR");

        Launcher = hardwareMap.get(Servo.class, "Launcher");

        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }


    @Override
    public void loop() {
        if (gamepad1.b && !lastA) {
            autoAlign = !autoAlign;
        }
        lastA = gamepad1.b;
        if (autoAlign) {
            RunAprilTagAuto();
        } else  {
            double leftPower, rightPower;

            double forward = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            leftPower = Range.clip(forward - rotate, -1.0, 1.0);
            rightPower = Range.clip(forward + rotate, -1.0, 1.0);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        if (!ShooterOn && !LauncherLock) { // So if Shooter is not on and LaunchLock is false, LaunchLock will be true, which makes the Launcher not work;
            LauncherLock = true;
        }
         else if (gamepad1.a && ShooterOn) { // And if gamepad1.a is pressed && the Shoter is active you can use the Launcher
            LauncherLock = false;
            Launcher.setPosition(0.5);
            sleep(300);
            Launcher.setPosition(0.875);
        }

        if (gamepad1.y && !lastY) {
            ShooterOn = !ShooterOn;
        }
        lastY = gamepad1.y;

        if (gamepad1.dpadRightWasPressed()) {ShooterVel = 18500;}

        if (gamepad1.dpadLeftWasPressed()) {ShooterVel = 1500;}

        if (gamepad1.dpadUpWasPressed()){ShooterVel += 50;}

        if(gamepad1.dpadDownWasPressed()) {ShooterVel -= 50;}

        if (ShooterOn) {
            flyWheel.setVelocity(ShooterVel);
        }else {
            flyWheel.setVelocity(0);
        }

        if (flyWheel.getVelocity() > ShooterVel) {
            gamepad1.rumble(300);
        }

        telemetry.addLine("===== TELEOP STATUS =====");
        telemetry.addData("MODE", autoAlign ? "AUTO ALIGN": "MANUAL");
        telemetry.addData("Shooter Status", ShooterOn ? "ON" : "OFF");
        telemetry.addData("Launcher Status", LauncherLock ? "OFF" : "ON");
        telemetry.addData("Left Power", "%.2f", leftDrive.getPower());
        telemetry.addData("Right Power", "%.2f", rightDrive.getPower());
        telemetry.addData("Velocity","%.4f RPM", flyWheel.getVelocity());
        telemetry.addData("Target Vel", "%.4f RPM", ShooterVel);
        telemetry.update();
    }

    public void RunAprilTagAuto() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.isEmpty()) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            telemetry.addLine("MODE : AUTO | Tag Not Found");
            return;
        }

        AprilTagDetection tag = detections.get(0);
        double smallestBearing = Math.abs(tag.ftcPose.bearing);

        for (AprilTagDetection detection: detections ) {
            double bearing = Math.abs(detection.ftcPose.bearing);
            if (bearing < smallestBearing) {
                smallestBearing = bearing;
                tag = detection;
            }
        }

        double TargetError = tag.ftcPose.range - TARGET_DISTANCE;
        double YawError = tag.ftcPose.yaw;
        double bearingError = tag.ftcPose.bearing;

        double drivePower = 0;
        double turnPower = (bearingError * kP_Bearing) + (YawError * kP_Yaw);

        if (Math.abs(TargetError) > 2.0) { // 2.0 Inch
            drivePower = Range.clip(TargetError * kP_Drive, -0.6, 0.6);
        }

        turnPower = Range.clip(turnPower, -0.4, 0.4);

        double leftPower = drivePower - turnPower;
        double rightPower = drivePower + turnPower;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        if (Math.abs(TargetError) <= 2.0 && Math.abs(YawError) < 10.0 && Math.abs(bearingError) < 2.0) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        telemetry.addLine("AUTO ALIGN ACTIVE");
        telemetry.addData("Range (inch)", "%.1f", tag.ftcPose.range);
        telemetry.addData("Bearing", "%.2f", bearingError);
        telemetry.addData("Yaw", "%.2f", YawError);
    }

    @Override
    public void stop() {
        visionPortal.close();
    }
}

