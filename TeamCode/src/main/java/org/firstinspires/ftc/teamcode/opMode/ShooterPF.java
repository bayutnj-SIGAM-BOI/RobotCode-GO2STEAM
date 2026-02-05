package org.firstinspires.ftc.teamcode.opMode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Shooter PIDF Tuning")
public class ShooterPF extends OpMode {

    DcMotorEx flyWheel;
    private Servo Launcher;


    public static double kP = 12.65;
    public static double kF = 200;
    public static double targetRPM = 1800;

    boolean lastY = false;
    boolean lastA = false;
    boolean LauncherLock = true;

    boolean ShooterOn = false;
    PIDFCoefficients lastPIDF;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Launcher = hardwareMap.get(Servo.class, "Launcher");
        Launcher.setPosition(0.875);

        lastPIDF = new PIDFCoefficients(kP, 0 ,0 , kF);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, lastPIDF);
    }

    @Override
    public void loop() {

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

        PIDFCoefficients current = new PIDFCoefficients(kP, 0 ,0 , kF);
        if (!current.equals(lastPIDF)) {
            flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, current);
            lastPIDF = current;
        }

        if (ShooterOn) {
            flyWheel.setVelocity(targetRPM);
        } else {
            flyWheel.setVelocity(0);
        }



        packet.put("Target RPM", targetRPM);
        packet.put("Velocity", flyWheel.getVelocity());
        packet.put("Error", targetRPM - flyWheel.getVelocity());
        packet.put("kP", kP);
        packet.put("kF", kF);

// Status boolean boleh (tapi nggak ke-graph)
        packet.put("ShooterOn", ShooterOn ? 1 : 0);

// Kirim ke Dashboard
//        dashboard.sendTelemetryPacket(packet);
    }
}
