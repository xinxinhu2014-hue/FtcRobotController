package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ReleaseDoors;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherControl;
import org.firstinspires.ftc.teamcode.mechanisms.YawControl;
import org.firstinspires.ftc.teamcode.mechanisms.WallControl;

@Autonomous
public class AutoBlueSmallZone extends LinearOpMode {

    private static final int SETTLE_LOOPS = 6;    // how many consecutive loops inside tolerance before stopping
    MecanumDrive drive = new MecanumDrive();
    ReleaseDoors gates = new ReleaseDoors();
    IntakeControl intake = new IntakeControl();
    LauncherControl launch = new LauncherControl();
    YawControl robotYaw = new YawControl();
    WallControl walls = new WallControl();
    boolean towardRight = true;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        gates.init(hardwareMap);
        intake.init(hardwareMap);
        walls.init(hardwareMap);
        launch.init(hardwareMap);
        robotYaw.init(hardwareMap);
        robotYaw.resetYaw();

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

    private void driveForwardInchesVel(double inches, double baseRPM, double targetDeg, double timeoutSec) {
        double baseVelTps = drive.forwardRunToTargetPosition(inches, baseRPM); // set target position and get base velocity in TPS ready
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec && drive.isMotorBusy()) {
            double yawErr = targetDeg - robotYaw.getYaw();
            drive.forwardAdjustYawError(yawErr, baseVelTps); // give each wheel different adjusted velocity based on yaw error
            telemetry.addData("Mode", "Straight");
            telemetry.addData("Target - Yaw Error", "%.1f - %.1f", targetDeg, yawErr);
            telemetry.update();
        }
        drive.stopDrive();
        drive.runUsingEncoders();
    }

    private void driveStrafeInchesVel(double inches, double baseRPM, double targetDeg, double timeoutSec, boolean right) {
        double baseVelTps = drive.strafeRunToTargetPosition(inches, baseRPM, right); // set target position and get base velocity in TPS ready
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec && drive.isMotorBusy()) {
            double yawErr = targetDeg - robotYaw.getYaw();
            drive.strafeAdjustYawError(yawErr, baseVelTps); // give each wheel different adjusted velocity based on yaw error
            telemetry.addData("Mode", "Strafe" + (right ? "Right" : "Left"));
            telemetry.addData("Target - Yaw Error", "%.1f - %.1f", targetDeg, yawErr);
            telemetry.update();
        }
        drive.stopDrive();
        drive.runUsingEncoders();
    }

    private void turnToHeadingDeg(double targetDeg, double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        int settled = 0;
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double yawErr = targetDeg - robotYaw.getYaw();
            // Stop adjusting, if turned into tolerance range of target and it has been adjusted enough time (SETTLE_LOOPS).
            if (drive.turnAdjustYawErr(yawErr) && ++settled >= SETTLE_LOOPS) break;
            telemetry.addData("Target - Yaw Error", "%.1f - %.1f", targetDeg, yawErr);
            telemetry.update();
        }
        drive.stopDrive();
        drive.runUsingEncoders();
    }

    private void turnByDeg(double deltaDeg, double timeoutSec) {
        double start = robotYaw.getYaw();
        double target = drive.setHeadingDeg(start, deltaDeg);
        turnToHeadingDeg(target, timeoutSec);
    }

    private void shooting(double wheelTargetRpm) {
        launch.useVelocityControl(wheelTargetRpm);
        walls.tightenWall(0.12, 0.2);
        ElapsedTime spin = new ElapsedTime();
        spin.reset();
        while (opModeIsActive() && spin.seconds() < 8.0) {
            sleep(6000);
            double launchSpeed;
            for (int i = 0; i < 3 && opModeIsActive(); i++) {
                launchSpeed = launch.currentWheelRpm();
                telemetry.addData("launch target speed: ", wheelTargetRpm);
                telemetry.addData("Actual wheel RPM", "%.0f", launchSpeed);
                telemetry.addData("Ready to fire ball", i + 1);
                telemetry.update();

                gates.openDoor(0.6, 0.7);
                sleep(400);
                gates.closeDoor(0.1, 0.0);
                if (i < 2) {
                    intake.setIntakePower(1.0);
                    sleep(800);
                    intake.setIntakePower(0.0);
                    sleep(1000);
                }

                telemetry.addData("Ball fired", i + 1);
                telemetry.update();
            }
            launch.stopLaunch();
        }
    }
}