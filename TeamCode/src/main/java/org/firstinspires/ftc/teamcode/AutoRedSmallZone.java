package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Auto Red Small Zone", group = "Competition")
public class AutoRedSmallZone extends AutoDrive {

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

        straightInchesVel(-3, 0.1, target, 2.0);
        sleep(20);
        turnByDeg(-19, 2.0, 0.2);
        shooting(4250.0, 300, 0);
        sleep(200);

        turnByDeg(19, 2.0, 0.4);
        sleep(20);
        straightInchesVel(-22, 0.3, target-5, 2.0);
        sleep(20);
        turnByDeg(90, 2.0, 0.4);
        sleep(20);
        intaking(44, 0.3, target + 85, 2.5, 500);

        straightInchesVel(-45, 0.3, target+87, 2.0);
        sleep(20);
        turnByDeg(-90, 2.0, 0.4);
        straightInchesVel(22, 0.3, target, 2.0);
        sleep(20);
        turnByDeg(-19, 2.0, 0.4);
        shooting(4400.0, 0, 0);
        sleep(200);

        //turnByDeg(20, 2.0, 0.4);
        //sleep(20);
        straightInchesVel(-45, 0.4, target-5, 2.0);
        sleep(20);
        turnByDeg(90, 2.0, 0.4);
        sleep(20);
        intaking(42, 0.3, target + 87, 2.5, 100);
        straightInchesVel(-10, 0.3, target+87, 2.0);
        sleep(2500);
    }
}