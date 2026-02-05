package org.firstinspires.ftc.teamcode.opMode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp
public class AutoAlignPD extends OpMode {
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // PD Controller
    public static double kP = 0.1;
    public static double kD = 0.3;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.4;
    double curTime = 0;
    double lastTime = 0;

    // Driving Setup
    double rotate, forward;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    //
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        leftDrive = hardwareMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_Drive");

        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        double leftPower, rightPower;

         forward = -gamepad1.left_stick_y;
         rotate = gamepad1.right_stick_x;

        //get Apriltag Info
        aprilTagWebcam.update();
        AprilTagDetection id = aprilTagWebcam.getTagBySpesificId(21);

        if (gamepad1.right_bumper) {
            if (id != null) {
                error = goalX - id.ftcPose.bearing;

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -0.5, 0.5);


                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        leftPower = Range.clip(forward - rotate, -1.0, 1.0);
        rightPower = Range.clip(forward + rotate, -1.0, 1.0);

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        packet.put("kP", kP);
        packet.put("kD", kD);
        packet.put("error", error);
        packet.put("Target error", goalX);
        dashboard.sendTelemetryPacket(packet);

    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
