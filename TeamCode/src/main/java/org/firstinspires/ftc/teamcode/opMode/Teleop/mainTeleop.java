package org.firstinspires.ftc.teamcode.opMode.Teleop;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Teleop", group = "Go2Steam")
public class mainTeleop extends OpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx flyWheel = null;
    private Servo Launcher;
    boolean autoAlign = false;
    double targetRPM = 1800;
    boolean lastA = false;
    boolean lastY = false;
    boolean ShooterOn = false;
    boolean LauncherLock = true;
    PIDFCoefficients lastPIDF;
    public static double kP = 200;
    public static double kF = 12.6;

    // Auto align
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // PD Controller
    public static double kPAuto = 0.1;
    public static double kD = 0.3;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = 0.4;
    double curTime = 0;
    double lastTime = 0;

    // Driving Setup
    double rotate, forward;

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

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Launcher.setPosition(0.875);
        lastPIDF = new PIDFCoefficients(kP, 0, 0, kF);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lastPIDF);
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
        double speedMultiply = gamepad1.right_bumper ? 0.6 : 1;

        //get Apriltag Info
        aprilTagWebcam.update();
        AprilTagDetection id = aprilTagWebcam.getTagBySpesificId(21);

        if (gamepad1.left_trigger > 0.3) {
            if (id != null) {
                error = goalX - id.ftcPose.bearing;

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kPAuto;

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

        leftDrive.setPower(leftPower * speedMultiply);
        rightDrive.setPower(rightPower * speedMultiply);


        // Shooter
        if (!ShooterOn && !LauncherLock) { // So if Shooter is not on and LaunchLock is false, LaunchLock will be true, which makes the Launcher not work;
            LauncherLock = true;
        } else if (gamepad1.a && ShooterOn) { // And if gamepad1.a is pressed && the Shoter is active you can use the Launcher
            LauncherLock = false;
            Launcher.setPosition(0.5);
            sleep(300);
            Launcher.setPosition(0.875);
        }

        if (gamepad1.y && !lastY) {
            ShooterOn = !ShooterOn;
        }
        lastY = gamepad1.y;


        if (gamepad1.dpadLeftWasPressed()) {
            targetRPM = 1500;
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetRPM += 50
            ;
        }

        if (gamepad1.dpadDownWasPressed()) {
            targetRPM -= 50;
        }


        PIDFCoefficients current = new PIDFCoefficients(kP, 0, 0, kF);
        if (!current.equals(lastPIDF)) {
            flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, current);
            lastPIDF = current;
        }
        targetRPM = Range.clip(targetRPM, 1400, 1800);

        if (ShooterOn) {
            flyWheel.setVelocity(targetRPM);
        } else {
            flyWheel.setVelocity(0);
        }

        telemetry.addLine("===== TELEOP STATUS =====");
        telemetry.addData("MODE", autoAlign ? "AUTO ALIGN" : "MANUAL");
        telemetry.addData("Shooter Status", ShooterOn ? "ON" : "OFF");
        telemetry.addData("Launcher Status", LauncherLock ? "OFF" : "ON");
        telemetry.addData("Left Power", "%.2f", leftDrive.getPower());
        telemetry.addData("Right Power", "%.2f", rightDrive.getPower());
        telemetry.addData("Target Vel", "%.2f", targetRPM);
        telemetry.addData("kF", kF);
        telemetry.addData("Velocity", "%.4f RPM", flyWheel.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}

