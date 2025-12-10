package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ReleaseDoors;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherControl;
import org.firstinspires.ftc.teamcode.mechanisms.YawControl;
import org.firstinspires.ftc.teamcode.mechanisms.WallControl;

/**
 * Base class for all autonomous OpModes that need drive + shooter utilities.
 */
public abstract class AutoDrive extends LinearOpMode {

    // ***** Shared hardware *****
    protected static final int SETTLE_LOOPS = 6;    // how many consecutive loops inside tolerance before stopping

    protected MecanumDrive drive   = new MecanumDrive();
    protected ReleaseDoors gates   = new ReleaseDoors();
    protected IntakeControl intake = new IntakeControl();
    protected LauncherControl launch = new LauncherControl();
    protected YawControl robotYaw  = new YawControl();
    protected WallControl walls    = new WallControl();

    // You can let each OpMode set this as needed
    protected boolean towardRight = true;

    @Override
    public abstract void runOpMode() throws InterruptedException;

    /**
     * Call this at the start of your child OpMode's runOpMode().
     */
    protected void initBaseHardware() {
        drive.init(hardwareMap);
        gates.init(hardwareMap);
        walls.init(hardwareMap);
        intake.init(hardwareMap);
        launch.init(hardwareMap);
        robotYaw.init(hardwareMap);

        // Default start positions – same as your old code
        gates.closeDoor(0.1, 0.2);
        walls.loosenWall(0.74, 0.7);
    }

    // =========================================================
    //  Shared drive helper methods
    // =========================================================

    protected void driveForwardInchesVel(double inches, double baseRPM, double targetDeg, double timeoutSec) {
        double baseVelTps = drive.forwardRunToTargetPosition(inches, baseRPM); // set target position and get base velocity in TPS ready
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec && drive.isMotorBusy()) {
            double yawErr = targetDeg - robotYaw.getYaw();
            drive.forwardAdjustYawError(yawErr, baseVelTps); // give each wheel different adjusted velocity based on yaw error
            telemetry.addData("Mode", "Straight");
            telemetry.addData("Target - Yaw Error", "%.1f - %.1f", targetDeg, yawErr);
            telemetry.update();
            idle();
        }
        drive.stopDrive();
        drive.runUsingEncoders();
    }

    protected void driveStrafeInchesVel(double inches, double baseRPM, double targetDeg, double timeoutSec, boolean right) {
        double baseVelTps = drive.strafeRunToTargetPosition(inches, baseRPM, right); // set target position and get base velocity in TPS ready
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec && drive.isMotorBusy()) {
            double yawErr = targetDeg - robotYaw.getYaw();
            drive.strafeAdjustYawError(yawErr, baseVelTps); // give each wheel different adjusted velocity based on yaw error
            telemetry.addData("Mode", "Strafe" + (right ? "Right" : "Left"));
            telemetry.addData("Target - Yaw Error", "%.1f - %.1f", targetDeg, yawErr);
            telemetry.update();
            idle();
        }
        drive.stopDrive();
        drive.runUsingEncoders();
    }

    protected void turnToHeadingDeg(double targetDeg, double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        int settled = 0;
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double yawErr = targetDeg - robotYaw.getYaw();
            // Stop adjusting if turned into tolerance range of target and it has been adjusted enough times (SETTLE_LOOPS)
            if (drive.turnAdjustYawErr(yawErr)) {
                settled++;
                if (settled >= SETTLE_LOOPS) break;
            } else {
                settled = 0; // lost tolerance; start counting again
            }

            telemetry.addData("Target - Yaw Error", "%.1f - %.1f", targetDeg, yawErr);
            telemetry.update();
            idle();
        }
        drive.stopDrive();
        drive.runUsingEncoders();
    }

    protected void turnByDeg(double deltaDeg, double timeoutSec) {
        double start = robotYaw.getYaw();
        double target = drive.setHeadingDeg(start, deltaDeg);
        turnToHeadingDeg(target, timeoutSec);
    }

    // =========================================================
    //  Shared shooting method
    // =========================================================

    protected void shooting(double wheelTargetRpm) {
        walls.tightenWall(0.16, 0.2);

        final double RPM_TOLERANCE = 75.0;
        final double TIMEOUT_SEC = 10.0;

        ElapsedTime spin = new ElapsedTime();
        spin.reset();

        // Spin up immediately
        launch.boostLaunch(1.0);
        sleep(600);
        launch.useVelocityControl(wheelTargetRpm);

        int ballsFired = 0;

        while (opModeIsActive() && spin.seconds() < TIMEOUT_SEC && ballsFired < 3) {
            double launchSpeed = launch.currentWheelRpm();
            double launchSpeedError = Math.abs(launchSpeed - wheelTargetRpm);

            // Wait until within tolerance
            if (launchSpeedError > RPM_TOLERANCE) {
                // Let PIDF keep working
                launch.useVelocityControl(wheelTargetRpm);
                sleep(50);
                continue;
            }

            telemetry.addData("Target RPM", wheelTargetRpm);
            telemetry.addData("Actual RPM", "%.0f", launchSpeed);
            telemetry.addData("RPM Error", "%.0f", launchSpeedError);
            telemetry.addData("Balls fired", ballsFired + 1);
            telemetry.update();

            double rollDownPower = -0.7;
            long rollDownTime = 500;
            double rollUpPower = 0.8;
            long rollUpTime = 250;

            // Now within tolerance: open door for ball 1 and 2
            if (ballsFired < 2) {
                gates.openDoor(0.6, 0.5);
                intake.setIntakePower(rollDownPower);
                sleep(rollDownTime);
                intake.setIntakePower(0.0);
                sleep(300); // short settle
            }

            // last ball (3rd): push up
            if (ballsFired == 2) {
                rollUpTime = 500;
                intake.setIntakePower(rollUpPower);
                sleep(rollUpTime);
            }

            ballsFired++;

            if (ballsFired == 1) {
                gates.closeDoor(0.1, 0.2);
                sleep(200); // a bit of time for gate to move
                intake.setIntakePower(rollUpPower);
                sleep(rollUpTime);
                intake.setIntakePower(0.0);
                sleep(100);
            }
        }

        // Safety: stop everything and reset
        intake.setIntakePower(0.0);
        launch.stopLaunch();
        gates.closeDoor(0.1, 0.2);
        walls.loosenWall(0.74, 0.7);
    }
}
