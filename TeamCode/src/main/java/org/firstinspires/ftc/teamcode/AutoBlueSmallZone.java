package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Auto Blue Small Zone", group = "Competition")
public class AutoBlueSmallZone extends AutoDrive {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize shared hardware from BaseAuto
        initBaseHardware();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        driveForwardInchesVel(3, drive.percentMaxRpm(0.25), 0.0, 2.0);
        sleep(50);
        //driveStrafeInchesVel(24, drive.percentMaxRpm(0.4), 0.0, 6, towardRight);
        //sleep(250);
        turnByDeg(15, 2.0);
        sleep(50);
        shooting(4400.0);
        turnToHeadingDeg(0.0, 2.0);
        sleep(50);
        driveForwardInchesVel(24, drive.percentMaxRpm(0.5), 0.0, 2.0);
        drive.stopDrive();
    }
}