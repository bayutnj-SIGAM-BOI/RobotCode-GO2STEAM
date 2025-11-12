package org.firstinspires.ftc.teamcode.opMode.AutonomousRoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous(name="Autonomous_1", group ="Go2Steam")
public class opMode extends LinearOpMode {

//    Shooter aShooter = new Shooter();

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(56, 0, Math.toRadians(-90));


        TankDrive drive = new TankDrive(hardwareMap, beginPose);
//        aShooter.init(hardwareMap);


        waitForStart();
        telemetry.addData("Status", "Autonomous Start");
        telemetry.update();

        if (isStopRequested()) return;
        {

//            Actions
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToX(-7)
                            .turn(Math.toRadians(-130))
                            .waitSeconds(2)
//                            Put Shooter mechanism here
                            .lineToY(60)
                            .build()
            );
        }
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }
}
