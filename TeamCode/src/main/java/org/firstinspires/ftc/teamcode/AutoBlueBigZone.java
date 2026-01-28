package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Auto Blue Big Zone", group = "Competition")
public class AutoBlueBigZone extends AutoDrive {


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize shared hardware from BaseAuto
        initBaseHardware();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        robotYaw.resetYaw();


        driveForwardInchesVel(30, drive.percentMaxRpm(0.5), 0.0, 0.8,2.0);
        sleep(50);
        turnByDeg(-125, 2.0, 0.3);
        sleep(50);
        driveForwardInchesVel(-10, drive.percentMaxRpm(0.5), 125, 0.8,2.0);
        sleep(50);
        shooting(3300.0, 100, 0);
        sleep(500);
        turnToHeadingDeg(0.0, 2.5, 0.3);
        sleep(50);
        driveForwardInchesVel(50, drive.percentMaxRpm(0.5), 0.0, 0.8,2.0);
        drive.stopDrive();

        //intaking(-20, drive.percentMaxRpm(0.2), 0.0, 2.0, 50);
    }
}