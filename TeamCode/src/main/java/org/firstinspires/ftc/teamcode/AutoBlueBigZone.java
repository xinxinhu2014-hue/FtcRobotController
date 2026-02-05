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

        double target = robotYaw.getYaw();  // capture once


        straightInchesVel(26, 0.3, target, 2.0);
        sleep(20);
        turnByDeg(45, 2.0, 0.4);
        sleep(20);

        shooting(3450.0, 100, 475);
        sleep(200);

        turnToHeadingDeg(target, 2.5, 0.4);
        sleep(20);
        straightInchesVel(19, 0.3, target, 2.0);
        sleep(20);
        turnToHeadingDeg(target - 87, 2.5, 0.4);
        sleep(20);
        intaking(31, 0.3, target - 87, 2.5, 1000);
        straightInchesVel(-35, 0.3, target - 87, 3.0);
        sleep(20);
        turnByDeg(130, 2.0, 0.4);

        shooting(3500.0, 150, 150);
        sleep(200);

        turnToHeadingDeg(target, 2.5, 0.4);
        sleep(20);
        straightInchesVel(18, 0.3, target, 2.0);
        sleep(20);
        turnToHeadingDeg(target - 87, 2.5, 0.4);
        sleep(20);
        intaking(48, 0.3, target - 87, 2.5, 500);
        loosenWall(0.84, 0.80);
        //gates.closeDoor(0.18, 0.31);
        straightInchesVel(-20, 0.3, target - 87, 2.0);

        //intaking(-20, drive.percentMaxRpm(0.2), 0.0, 2.0, 50);
    }
}