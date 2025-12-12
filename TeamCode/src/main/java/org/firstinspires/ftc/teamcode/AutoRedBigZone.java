package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous (name = "Auto Red Big Zone", group = "Competition")
public class AutoRedBigZone extends AutoDrive {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize shared hardware from BaseAuto
        initBaseHardware();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        robotYaw.resetYaw();

        driveForwardInchesVel(30, drive.percentMaxRpm(0.5), 0.0, 2.0);
        sleep(50);
        turnByDeg(125, 2.0);
        sleep(50);
        driveForwardInchesVel(-10, drive.percentMaxRpm(0.5), 125, 2.0);
        sleep(50);
        shooting(3200.0, 200, 200);
        turnToHeadingDeg(0.0, 2.5);
        sleep(50);
        driveForwardInchesVel(30, drive.percentMaxRpm(0.5), 0.0, 2.0);
        drive.stopDrive();
    }
}
