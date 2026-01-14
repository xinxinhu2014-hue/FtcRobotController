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
    protected double leftGateClosePosition = 0.22, rightGateClosePosition = 0.35, leftGateOpenPosition = 0.6, rightGateOpenPosition = 0.5;

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
        gates.closeDoor(leftGateClosePosition, rightGateClosePosition);
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
    //  Shared shooting method - 3 Ball Sequential Shooting
    // =========================================================

    protected void shooting(double wheelTargetRpm, double ballTwoRpmAdjust, double ballThreeRpmAdjust) {
        walls.tightenWall(0.16, 0.2);

        double RPM_TOLERANCE = 0.05 * wheelTargetRpm;
        final double TIMEOUT_SEC = 10.0;




        // Boost phase
        //launch.boostLaunch(1.0);
        //sleep(600);

        // Switch to velocity control
        //launch.useVelocityControl(wheelTargetRpm);

        int ballCount = 0;
        ElapsedTime launchingTimer = new ElapsedTime();
        launchingTimer.reset();

        while (opModeIsActive() && launchingTimer.seconds() < TIMEOUT_SEC && ballCount < 3) {
            double launchRpm = launch.currentWheelRpm();
            double launchRpmError =wheelTargetRpm - launchRpm;

            telemetry.addData("Target RPM", wheelTargetRpm);
            /*
            telemetry.addData("Actual RPM", "%.0f", launchRpm);
            if (launchRpmError < 0) {
                telemetry.addData("Above target by", "%.0f", -launchRpmError);
            } else {
                telemetry.addData("Below target by", "%.0f", launchRpmError);
            }
            telemetry.update();
             */

            // Wait until within tolerance
            if (launchRpmError > 400) {
                launch.boostLaunch(1.0);
                sleep(20);
                telemetry.addData("Actual RPM", "%.0f", launchRpm);
                telemetry.addData("Below target by", "%.0f", launchRpmError);
                telemetry.update();
                continue;
            } else if (Math.abs(launchRpmError) > RPM_TOLERANCE) {
                launch.useVelocityControl(wheelTargetRpm);
                sleep(20);
                telemetry.addData("Actual RPM", "%.0f", launchRpm);
                if (launchRpmError < 0) {
                    telemetry.addData("Above target by", "%.0f", -launchRpmError);
                } else {
                    telemetry.addData("Below target by", "%.0f", launchRpmError);
                }
                telemetry.update();
                continue;
            }


            telemetry.addData("Ball", ballCount + 1);

            // Ball 1: Open gate, pause, close gate, roll up
            if (ballCount == 0) {
                gates.openDoor(leftGateOpenPosition, rightGateOpenPosition);
                sleep(500);  // pause for gate to open
                ballCount++;
/*
                gates.closeDoor(leftGateClosePosition, rightGateClosePosition);
                sleep(100);  // pause for gate to close

                intake.setIntakePower(1.0);  // nudge 2nd ball behind closed gate
                sleep(100);
                intake.setIntakePower(0.0);

 */

                // Adjust RPM for 2nd ball
                wheelTargetRpm += ballTwoRpmAdjust;
                RPM_TOLERANCE = 0.05 * wheelTargetRpm;
                launch.useVelocityControl(wheelTargetRpm);
                sleep(200);  // wait for wheel recovery
            }
            // Ball 2: Open gate, roll down, close gate
            else if (ballCount == 1) {
                //gates.openDoor(leftGateOpenPosition, rightGateOpenPosition);
                intake.setIntakePower(1.0);  // roll 3rd ball down
                sleep(300);
                //intake.setIntakePower(0.0);
                ballCount++;

                // No RPM adjustment for 3rd ball (or adjust as needed)
                wheelTargetRpm += ballThreeRpmAdjust;
                RPM_TOLERANCE = 0.05 * wheelTargetRpm;
                launch.useVelocityControl(wheelTargetRpm);
                walls.loosenWall(0.74, 0.7);
                sleep(200);  // wait for wheel recovery
            }
            // Ball 3: Roll up and shoot
            else if (ballCount == 2) {
                walls.tightenWall(0.16, 0.2);
                intake.setIntakePower(1.0);  // roll 3rd ball up
                sleep(1000);
                intake.setIntakePower(0.0);
                ballCount++;
            }
        }

        // Reset everything
        intake.setIntakePower(0.0);
        launch.stopLaunch();
        gates.closeDoor(leftGateClosePosition, rightGateClosePosition);
        walls.loosenWall(0.74, 0.7);
    }
}
