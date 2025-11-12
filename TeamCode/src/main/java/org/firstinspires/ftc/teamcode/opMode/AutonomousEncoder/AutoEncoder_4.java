package org.firstinspires.ftc.teamcode.opMode.AutonomousEncoder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opMode.AutonomousEncoder.Components.ShootArtifactCode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Auto_4", group = "Go2Steam")
public class AutoEncoder_4 extends LinearOpMode {
    ShootArtifactCode ArtifactShoot = new ShootArtifactCode();


    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private ElapsedTime runtime = new ElapsedTime();

    //    Webcam
    private static final boolean WEBCAM = true;

    private static final int Desired_Tag_Id = -1;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagDetection DesiredTag = null;


    @Override
    public void runOpMode(){
        ArtifactShoot.init(hardwareMap);
        AprilTagInit();

        leftDrive = hardwareMap.get(DcMotor.class, "left_motor_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_drive");
//        set Direction to Reverse
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

//        Encoder Setup
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
//        Code Here
        ThreeArtifactShooters();
        driveEncoders(-1500, -1500, 0.60, 2);
        sleep(2500);
        driveEncoders(1500, 1500, 0.60, 2);
        ThreeArtifactShooters();

//        Detecting the ID
        while (opModeIsActive()) {
            boolean targetFound = false;
            DesiredTag = null;

            List<AprilTagDetection> currentDetection = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetection) {
                if (detection.metadata != null) {
                    if (Desired_Tag_Id < 0 || (detection.id) == Desired_Tag_Id) {
                        targetFound = true;
                        DesiredTag = detection;
                        break;
                    } else if (DesiredTag.id != Desired_Tag_Id){
                        telemetry.addData("Unknown Tag", "Tag Id %d is not TagLibrary", detection.id);
                    } else {
                        telemetry.addData("Skipping Tag", "Tag Id %d is not Desired", detection.id);
                    }
                    if(targetFound) {
                        telemetry.addData("Target is Found", "Id %d (%s)", DesiredTag.id, DesiredTag.metadata.name);
                        telemetry.addData("Range", "%5.if Inches", DesiredTag.ftcPose.range);
                    }
                    telemetry.update();
                }
            }
        }
    }

//    setup Movement with Encoder

    public void driveEncoders(double leftTarget, double rightTarget, double speed, long ms) {
        int newleftTarget;
        int newrightTarget;

        if (opModeIsActive()) {
            newleftTarget = leftDrive.getCurrentPosition() + (int)(leftTarget);
            newrightTarget = rightDrive.getCurrentPosition() + (int)(rightTarget);

//            Start pos
            leftDrive.setTargetPosition(newleftTarget);
            rightDrive.setTargetPosition(newrightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() && runtime.seconds() <ms && leftDrive.isBusy() && rightDrive.isBusy()) {
                telemetry.addData("Running to", " %7d :%7d", newleftTarget, newrightTarget);
                telemetry.addData("Current Position", " %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }

//    Setup Shoot 3 artifact in once

    public void ThreeArtifactShooters() {
        int artifact = 3;
        while (opModeIsActive() && artifact > 0) {
            ArtifactShoot.shootMechanism();
            artifact -= 1;
            sleep(200);
        }
    }

    //        AprilTag init
    private void AprilTagInit() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .build();

        aprilTagProcessor.setDecimation(2);

        if (WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam_1"))
                    .addProcessor(aprilTagProcessor)
                    .build();
        }else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTagProcessor)
                    .build();
        }
    }
}


