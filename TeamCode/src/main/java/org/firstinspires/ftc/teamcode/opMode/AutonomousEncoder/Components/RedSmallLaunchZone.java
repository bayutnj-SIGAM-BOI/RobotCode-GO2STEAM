package org.firstinspires.ftc.teamcode.opMode.AutonomousEncoder.Components;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedSmallLaunchZone", group = "Go2steam")
public class RedSmallLaunchZone extends LinearOpMode{

    private DcMotor leftDrive, rightDrive = null;
    private DcMotorEx flyWheel = null;
    private Servo Launcher;
    private final ElapsedTime runtime = new ElapsedTime();

//    Drive Train motors setting
    static final double WHEEL_DIAMETER_INCHES = 3.54331; // 90mm In Inches 3,5 or more
    static final double COUNTS_PER_MOTOR_REV = 288.0; // CoreHex counts per Rev//
    static final double DRIVE_GEAR_REDUCTION = 1.0; // use no gear ration added//

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI); // Formula to convert ticks to inches FROM Google

//    FLyWheels setting
    static final double FLYWHEEL_TARGET_RPM = 3600; // Shooter wheel target RPM
    static final double FLYWHEEL_COUNTS_PER_REV= 28.0 * 3.0; // 28.0 is t he REV HDX Motor Revolution * 3.0
    static final double FLYWHEEL_TPS = (FLYWHEEL_TARGET_RPM * FLYWHEEL_COUNTS_PER_REV) / 60.0; // flywheel target RPM * Counts per revolution from the motor spec and divide by 60.0, that mean a minute


    @Override
    public void runOpMode(){
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor_Drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor_Drive");

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        Launcher = hardwareMap.get(Servo.class, "Launcher");


//        set Direction to Reverse
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        Encoder Setup
        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Launcher.setPosition(0.75);




//        Code Here
//
        driveEncoders(16, -16, 0.50, 1); //Turn right 24inches
        Shoot3x(); //Shoot the 3artifacts at once
        driveEncoders(16, -16, 0.50, 1); //Turn right 24inches to face the loading zone
        driveEncoders(72, 72, 0.90, 3); //move forward 72 inches to zone
        sleep(5000); //waiting human player to throw the artifact into the robot
        driveEncoders( -72,-72, 0.90, 3); // Go back to the small launch zone for 72 inches
        driveEncoders(  -16, 16, 0.70, 1); // turn left to face the goal for shooting the balls
        Shoot3x(); //shoot 3 artifact at once

        telemetry.addData("Path", "Complete");
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
         sleep(500);

         flyWheel.setVelocity(FLYWHEEL_TPS);
         Launcher.setPosition(0.3);
         sleep(delayShoot);

         sleep(500);

         Launcher.setPosition(0.75);
         flyWheel.setVelocity(0);


         telemetry.addData("Flywheel Velocity : ", flyWheel.getVelocity());
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