package org.firstinspires.ftc.teamcode.opMode.AutonomousEncoder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoEncoderPos1", group = "Go2Steam")
public class RedSmallLaunchZone extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotorEx flyWheelL, flyWheelR;
    private Servo artifactStooper, artifactPusher ;
    private ElapsedTime runtime = new ElapsedTime();

//    Drive Train motors setting
    static final double WHEEL_DIAMETER_INCHES = 100 / 25.4; // Or about 3 //
    static final double COUNTS_PER_MOTOR_REV = 288.0; // CoreHex counts per Rev//
    static final double DRIVE_GEAR_REDUCTION = 1.0; // OR use none//

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

//    FLyWheels setting
    static final double FLYWHEEL_TARGET_RPM = 3600;
    static final double FLYWHEEL_COUNTS_PER_REV= 28.0 * 3.0;
    static final double FLYWHEEL_TPS = (FLYWHEEL_TARGET_RPM * FLYWHEEL_COUNTS_PER_REV) / 60.0;



    @Override
    public void runOpMode(){

        leftDrive = hardwareMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_Drive");

        flyWheelL = hardwareMap.get(DcMotorEx.class, "flyWheelL");
        flyWheelR = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        artifactStooper = hardwareMap.get(Servo.class, "artifactStooper");
        artifactPusher = hardwareMap.get(Servo.class, "artifactPusher");


//        set Direction to Reverse
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelR.setDirection(DcMotorSimple.Direction.REVERSE);

//        Encoder Setup
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        artifactPusher.setDirection(Servo.Direction.REVERSE);
        artifactStooper.setDirection(Servo.Direction.REVERSE);
        artifactStooper.setPosition(0.750);
        artifactPusher.setPosition(1.100);

        waitForStart();
//        Code Here
//
        driveEncoders(16, -16, 0.50, 1); //Turn right 24inches
        Shoot3x(); //Shoot the 3artifacts at once
        driveEncoders(16, -16, 0.50, 1); //Turn right 24inches to face the loading zone
        driveEncoders(72, 72, 0.90, 3); //move forward 72 inches to zone
        sleep(5000); //waiting the human player throw the artifact into the robot
        driveEncoders(-72,-72, 0.90, 3); // Go back to the small launch zone for 72 inches
        driveEncoders(-16, 16, 0.70, 1); // turn left to face the goal for shooting the balls
        Shoot3x(); //shoot 3 artifact at once

        telemetry.addData("Path", "Complate");
        telemetry.update();

        while(opModeIsActive()) {
            idle();
        }
    }

//    setup Movement with Encoder

    private void driveEncoders(double leftTarget, double rightTarget, double speed, long TimeoutSecs) {
        int newleftTarget;
        int newrightTarget;

        if (opModeIsActive()) {
            newleftTarget = leftDrive.getCurrentPosition() + (int)(leftTarget * COUNTS_PER_INCH);
            newrightTarget = rightDrive.getCurrentPosition() + (int)(rightTarget * COUNTS_PER_INCH);

//            Start pos
            leftDrive.setTargetPosition(newleftTarget);
            rightDrive.setTargetPosition(newrightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() && runtime.seconds() <TimeoutSecs && leftDrive.isBusy() && rightDrive.isBusy()) {
                telemetry.addData("Running to", " %7d :%7d", newleftTarget, newrightTarget);
                telemetry.addData("Current Position", " %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void Shooters(long delayShoot) {
        if (opModeIsActive()) {
         artifactPusher.setPosition(0.650);
         sleep(500);

         artifactStooper.setPosition(0.5);
         sleep(200);

         flyWheelR.setVelocity(FLYWHEEL_TPS);
         flyWheelL.setVelocity(FLYWHEEL_TPS);
         sleep(delayShoot);

         artifactPusher.setPosition(1.100);
         sleep(500);

         artifactStooper.setPosition(0.750);
         sleep(200);

         flyWheelR.setVelocity(0);
         flyWheelL.setVelocity(0);

         telemetry.addData("FlyL Velocity : ", flyWheelL.getVelocity());
         telemetry.addData("FlyR Velocity : ", flyWheelR.getVelocity());
         telemetry.update();
        }
    }

    private void Shoot3x() {
        for (int a = 0; a < 3; a++) {
            Shooters(300);
            sleep(300);
        }
    }
}