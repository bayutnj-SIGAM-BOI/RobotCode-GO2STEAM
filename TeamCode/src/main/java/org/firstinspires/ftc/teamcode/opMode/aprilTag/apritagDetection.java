package org.firstinspires.ftc.teamcode.opMode.aprilTag;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class apritagDetection {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private ArrayList<AprilTagDetection> detectedTag = new ArrayList<AprilTagDetection>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTag = aprilTagProcessor.getDetections();
    }

    public ArrayList<AprilTagDetection> getDetectedTag() {
        return  detectedTag;
    }

    public void dislpayTelemetry(AprilTagDetection detectionId) {
        if (detectionId == null) {return ;}
        if (detectionId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectionId.id, detectionId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectionId.ftcPose.x, detectionId.ftcPose.y, detectionId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectionId.ftcPose.pitch, detectionId.ftcPose.roll, detectionId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectionId.ftcPose.range, detectionId.ftcPose.bearing, detectionId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectionId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectionId.center.x, detectionId.center.y));
        }
    }

    public AprilTagDetection getTagBySpesificId(int id) {
        for (AprilTagDetection detection : detectedTag) {
            if (detection.id == id) {
                return detection;
            }
        }
        return  null;
    }

    public void stopDetect() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
