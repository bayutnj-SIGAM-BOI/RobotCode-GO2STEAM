package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;  // Made private and fixed assignment
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTag = new ArrayList<>();

    private Telemetry telemetry;

    /**
     * Initialize the AprilTag detection system
     * @param hardwareMap Robot hardware map
     * @param telemetry Telemetry for status updates
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Configure AprilTag processor with optimized settings
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setNumThreads(1)  // Single thread for efficiency
                .build();

        // Build vision portal with webcam
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam1"));
        builder.setCameraResolution(new Size(640, 480));  // Good balance of speed/accuracy
        builder.enableLiveView(false);  // Disable preview for better performance
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

        // Set manual exposure for consistent lighting
        setManualExposure(6, 240);

        telemetry.addData("AprilTag", "Initialized");
        telemetry.update();
    }

    /**
     * Set manual camera exposure and gain for consistent AprilTag detection
     * Lower exposure = less motion blur but needs more light
     * Higher gain = brighter image but more noise
     *
     * @param exposureMs Exposure time in milliseconds (recommended: 4-8)
     * @param gain Camera gain (recommended: 200-250)
     */
    private void setManualExposure(int exposureMs, int gain) {
        // Wait for camera to be ready
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream...");
            telemetry.update();

            // Fixed: Changed && to || in while condition
            while (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {
                }
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Apply manual exposure settings
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            try {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

                // Switch to manual mode if needed
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    Thread.sleep(50);
                }

                // Set exposure and gain
                exposureControl.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);
                Thread.sleep(20);
                gainControl.setGain(gain);
                Thread.sleep(20);

                telemetry.addData("Exposure", "%d ms", exposureMs);
                telemetry.addData("Gain", gain);

            } catch (Exception e) {
                telemetry.addData("Camera Control Error", e.getMessage());
            }
            telemetry.update();
        }
    }

    /**
     * Update the list of detected AprilTags
     * Call this in your OpMode loop before accessing detection data
     */
    public void update() {
        if (aprilTagProcessor != null) {
            detectedTag = aprilTagProcessor.getDetections();
        }
    }

    /**
     * Get all currently detected AprilTags
     * @return List of detected tags
     */
    public List<AprilTagDetection> getDetectedTag() {
        return detectedTag;
    }

    /**
     * Get a specific AprilTag by ID
     * @param id AprilTag ID to search for
     * @return AprilTagDetection if found, null otherwise
     */
    public AprilTagDetection getTagBySpesificId(int id) {
        for (AprilTagDetection detection : detectedTag) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Display detailed telemetry for a detected AprilTag
     * @param detection AprilTag detection to display
     */
    public void displayDetectionTelemetry(AprilTagDetection detection) {
        if (detection == null) {
            return;
        }

        if (detection.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (cm)",
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)",
                    detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (cm, deg, deg)",
                    detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)",
                    detection.center.x, detection.center.y));
        }
    }

    /**
     * Get the number of currently detected tags
     * @return Count of detected tags
     */
    public int getDetectedTagCount() {
        return detectedTag.size();
    }

    /**
     * Check if a specific tag ID is currently detected
     * @param id AprilTag ID to check
     * @return true if tag is detected, false otherwise
     */
    public boolean isTagDetected(int id) {
        return getTagBySpesificId(id) != null;
    }

    /**
     * Clean shutdown of vision system
     * Call this in your OpMode's stop() method
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}