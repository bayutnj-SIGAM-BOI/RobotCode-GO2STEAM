package org.firstinspires.ftc.teamcode.opMode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opMode.Teleop.Components.ShooterTunning;
import org.firstinspires.ftc.teamcode.opMode.Teleop.Components.Shooter_1;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "Teleop", group = "Go2Steam")
public class mainTeleop extends OpMode {
    Shooter_1 shooter = new Shooter_1();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    boolean autoAlign = false;

    double TARGET_DISTANCE = 144.0;
    static final double kP_Drive = 0.03;
    static final double kP_Bearing = 0.04;
    static final double kP_Yaw = 0.01;

    boolean lastA = false;

    @Override
    public void init() {

//        ================ Shooter initialize ================
        shooter.init(hardwareMap);



//        This is initialize the Drivetrain motors

//        ================ DriveTrain initialize ================
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_Drive");

        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
//                .setCameraResolution(new Size(720, 480))
                .build();
    }


    @Override
    public void loop() {
        //        ================ AutoAlign ================

        //        Gamepad 1 for Controlling Drive Train, and I use the Range. clip so the drivetrain speed is not more than 1.0 speed
//        ================ DriveTrain ================

        if (gamepad1.a && !lastA) {
            autoAlign = !autoAlign;
        }
        lastA = gamepad1.a;
        if (autoAlign) {
            RunAprilTagAuto();
        } else  {
            double leftPower, rightPower;

            double forward = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            leftPower = Range.clip(forward + rotate, -1.0, 1.0);
            rightPower = Range.clip(forward - rotate, -1.0, 1.0);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

//        Gamepad 2 for Controlling Shooter take from the Shooter components file

//        ================ Shooter ================
        double shootSpeed = gamepad2.right_bumper ? 0.6 : 1.0;
        double shoot = gamepad2.right_trigger * shootSpeed;
        double launcher = gamepad2.a ? 0.3 :  0.7;

        shooter.shootArtifact(shoot, launcher);


//        ================ Telemetry Information ================

        telemetry.addLine("===== TELEOP STATUS =====");
        telemetry.addData("MODE", autoAlign ? "AUTO ALIGN": "MANUAL");
        telemetry.addData("Left Power", "%.2f", leftDrive.getPower());
        telemetry.addData("Right Power", "%.2f", rightDrive.getPower());
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

        double leftPower = drivePower + turnPower;
        double rightPower = drivePower - turnPower;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        if (Math.abs(TargetError) <= 3.0 && Math.abs(YawError) < 0.5 && Math.abs(bearingError) < 0.5) { // 2.0 Inch, 1.5 Degree, 1.5 Bearing
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            for (int i = 0; i < 3; i++) {
                shooter.shootArtifact(1.0, 0.3);
            }
        }

        telemetry.addLine("AUTO ALIGN ACTIVE");
        telemetry.addData("Range (inch)", "%.1f", tag.ftcPose.range);
        telemetry.addData("Bearing", "%.2f", bearingError);
        telemetry.addData("Yaw", "%.2f", YawError);
    }
}

