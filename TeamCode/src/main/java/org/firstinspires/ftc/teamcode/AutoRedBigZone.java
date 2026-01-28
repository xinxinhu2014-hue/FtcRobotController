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
        sleep(200); // give the IMU a moment after reset

        double target = robotYaw.getYaw();  // capture once

        /*drive.runUsingEncoders();

        // Force a big L/R difference for 1 second
        double leftTps  = 0;
        double rightTps = 1000;

        drive.forwardAdjustYawError(0, 0); // optional; ignore

        // Direct set (bypass yaw math completely)
        drive.setRawLeftRightTps(leftTps, rightTps);  // add this helper below
        sleep(1000);
        drive.stopDrive();*/



        straightInchesVel(26, 0.3, target, 2.0);
        sleep(20);
        turnByDeg(-45, 2.0, 0.4);
        sleep(20);


        shooting(3350.0, -200, 375);
        sleep(200);

        turnToHeadingDeg(target, 2.5, 0.4);
        sleep(20);
        straightInchesVel(19, 0.3, target, 2.0);
        sleep(20);
        turnToHeadingDeg(target + 87, 2.5, 0.4);
        sleep(20);
        intaking(31, 0.3, target + 87, 2.5, 1000);
        straightInchesVel(-35, 0.3, target + 87, 3.0);
        sleep(20);
        turnByDeg(-130, 2.0, 0.4);

        shooting(3700.0, 0, 0);
        sleep(200);

        turnToHeadingDeg(target, 2.5, 0.4);
        sleep(20);
        straightInchesVel(18, 0.3, target, 2.0);
        sleep(20);
        turnToHeadingDeg(target + 87, 2.5, 0.4);
        sleep(20);
        intaking(55, 0.3, target + 87, 2.5, 1200);
        loosenWall(0.85, 0.81);
        straightInchesVel(-20, 0.3, target + 87, 2.0);

        //target = robotYaw.getYaw();
        //strafeInchesVel(26, 0.3, target, 2.5, true);
        /*turnToHeadingDeg(90, 2.5, 0.3);
        intaking(-30, drive.percentMaxRpm(0.3), -90, 2.5, 0.7);.
        sleep(50);
        driveForwardInchesVel(30, drive.percentMaxRpm(0.5), -90, 0.7, 2.5);
        turnToHeadingDeg(125, 2.5, 0.3);*/
        //drive.stopDrive();
    }
}