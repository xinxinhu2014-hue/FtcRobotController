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
    protected double leftGateClosePosition = 0.20, rightGateClosePosition = 0.33,
            leftGateOpenPosition = 0.75, rightGateOpenPosition = 0.62,
            leftWallLoosePosition = 0.88, rightWallLoosePosition = 0.84,
            leftWallTightPosition = 0.0, rightWallTightPosition = 0.04;

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
        walls.loosenWall(leftWallLoosePosition, rightWallLoosePosition);
    }

    // =========================================================
    //  Shared drive helper methods
    // =========================================================

    protected void driveForwardInchesVel(double inches, double baseRPM, double targetDeg, double leftAdjusst, double timeoutSec) {
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

    protected void turnToHeadingDeg(double targetDeg, double timeoutSec, double velPercent) {
        ElapsedTime timer = new ElapsedTime();
        int settled = 0;
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double yawErr = targetDeg - robotYaw.getYaw();
            // Stop adjusting if turned into tolerance range of target and it has been adjusted enough times (SETTLE_LOOPS)
            if (drive.turnAdjustYawErr(yawErr, velPercent)) {
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

    protected void turnByDeg(double deltaDeg, double timeoutSec, double velPercent) {
        double start = robotYaw.getYaw();
        double target = drive.setHeadingDeg(start, deltaDeg);
        turnToHeadingDeg(target, timeoutSec, velPercent);
    }

    // ==========================================================
    // Intake 3 balls
    // ==========================================================

    protected void intaking(double inches, double basePct, double targetDeg, double timeoutSec, long extraRunTime) {
        intake.setIntakePower(1.0);
        //walls.tightenWall(leftWallTightPosition , rightWallTightPosition); // test out position for tightening
        straightInchesVel(inches, basePct, targetDeg, timeoutSec);
        sleep(extraRunTime);
        intake.setIntakePower(0.0);
    }

    protected void loosenWall(double leftWallLoosePosition, double rightWallLoosePosition){
        walls.loosenWall(leftWallLoosePosition, rightWallLoosePosition);
    }

    // =========================================================
    //  Shared shooting method - 3 Ball Sequential Shooting
    // =========================================================

    protected void shooting(double wheelTargetRpm, double ballTwoRpmAdjust, double ballThreeRpmAdjust) {

        final double TIMEOUT_SEC = 12.0;
        final double BOOST_ON = 250;
        final double BOOST_OFF = 120;

        int ballCount = 0;
        double tol = 0.05 * wheelTargetRpm;
        boolean boosting = false;
        int settleCount = 0;


        ElapsedTime launchingTimer = new ElapsedTime();
        launchingTimer.reset();

        while (opModeIsActive() && launchingTimer.seconds() < TIMEOUT_SEC && ballCount < 3) {
            double launchRpm = launch.currentWheelRpm();
            double launchRpmError =wheelTargetRpm - launchRpm;

            telemetry.addData("Target RPM", wheelTargetRpm);

            telemetry.addData("Actual RPM", "%.0f", launchRpm);
            if (launchRpmError < 0) {
                telemetry.addData("Above target by", "%.0f", -launchRpmError);
            } else {
                telemetry.addData("Below target by", "%.0f", launchRpmError);
            }
            telemetry.update();


            // Wait until within tolerance
            if (launchRpmError > BOOST_ON) {
                if (!boosting) {
                    launch.boostLaunch(1.0);
                    boosting = true;
                }
                sleep(20);
                telemetry.addData("Actual RPM", "%.0f", launchRpm);
                telemetry.addData("Below target by", "%.0f", launchRpmError);
                telemetry.update();
                continue;
            }

            if (boosting && launchRpmError < BOOST_OFF) {
                launch.useVelocityControl(wheelTargetRpm);
                boosting = false;
            }

            if (Math.abs(launchRpmError) > tol) {
                sleep(20);
                telemetry.addData("Actual RPM", "%.0f", launchRpm);

                if (launchRpmError < 0) {
                    telemetry.addData("Above target by", "%.0f", -launchRpmError);
                } else {
                    telemetry.addData("Below target by", "%.0f", launchRpmError);
                }

                telemetry.update();
                continue;
            } else {
                settleCount++;
            }

            if (settleCount <= SETTLE_LOOPS) {
                continue;
            } else {
                settleCount = 0;
            }


            telemetry.addData("Ball", ballCount + 1);

            // Ball 1: Open gate, pause
            if (ballCount == 0) {
                gates.openDoor(leftGateOpenPosition, rightGateOpenPosition);
                sleep(300);  // pause for ball out
                ballCount++;


                // Adjust RPM for 2nd ball
                wheelTargetRpm += ballTwoRpmAdjust;
                tol = 0.05 * wheelTargetRpm;
                gates.closeDoor(leftGateClosePosition, rightGateClosePosition);
                //launch.useVelocityControl(wheelTargetRpm);
            }
            // Ball 2: Open gate, roll down, close gate
            else if (ballCount == 1) {
                //roll up to send ball 2 out, then pause
                intake.setIntakePower(1.0);
                gates.openDoor(leftGateOpenPosition, rightGateOpenPosition);
                sleep(800);
                ballCount++;

                // No RPM adjustment for 3rd ball (or adjust as needed)
                wheelTargetRpm += ballThreeRpmAdjust;
                tol = 0.05 * wheelTargetRpm;
                //gates.closeDoor(leftGateClosePosition, rightGateClosePosition);
                //launch.useVelocityControl(wheelTargetRpm);
            }
            // Ball 3: Roll up and shoot
            else if (ballCount == 2) {
                walls.tightenWall(leftWallTightPosition, rightWallTightPosition);
                sleep(1500);
                ballCount++;
            }


            /*
            gates.openDoor(leftGateOpenPosition, rightGateOpenPosition);
            sleep(200);
            intake.setIntakePower(1.0);
            sleep(2000);
            ballCount = 3;
             */
        }



        // Reset everything
        intake.setIntakePower(0.0);
        walls.loosenWall(leftWallLoosePosition, rightWallLoosePosition);
        gates.closeDoor(leftGateClosePosition, rightGateClosePosition);
        launch.stopLaunch();
    }

    protected void straightInchesVelSlowdown(
            double inches,
            double basePct,
            double targetDeg,
            double timeoutSec,
            double fracBrake,
            double minBrakeIn,
            double maxBrakeIn
    ) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        drive.cancelDistanceMove();
        drive.beginForwardDistanceInches(inches);

        boolean done = false;

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double currentYaw = robotYaw.getYaw();

            done = drive.updateForwardDistanceHoldHeadingSlowdown(
                    inches,
                    targetDeg,
                    currentYaw,
                    basePct,
                    0.12,
                    fracBrake,
                    minBrakeIn,
                    maxBrakeIn
            );

            // --- debug telemetry (use one source of truth: avgTicks) ---
            double avgTicks = drive.getAverageEncoderTicks();
            double calcInches = avgTicks / 28.52;

            telemetry.addData("t", "%.2f", timer.seconds());
            telemetry.addData("done", done);
            telemetry.addData("avgTicks", "%.0f", avgTicks);
            telemetry.addData("calcIn", "%.2f", calcInches);
            telemetry.addData("yawErr", "%.1f", targetDeg - currentYaw);
            telemetry.update();

            if (done) {
                drive.stopDriveVelocity();   // << use velocity stop
                break;
            }

            idle();
        }

        boolean timedOut = timer.seconds() >= timeoutSec;

        // final telemetry
        double avgTicks = drive.getAverageEncoderTicks();
        double calcInches = avgTicks / 28.52;
        telemetry.addData("EXIT", timedOut ? "TIMEOUT" : "DONE");
        telemetry.addData("finalTicks", "%.0f", avgTicks);
        telemetry.addData("finalCalcIn", "%.2f", calcInches);
        telemetry.update();

        // ✅ Only do anything extra if timed out (optional)
        if (opModeIsActive() && timedOut) {
            drive.stopDriveVelocity();
        }

        drive.cancelDistanceMove();
        drive.runUsingEncoders();
    }

    protected void straightInchesVel(
            double inches,
            double basePct,
            double targetDeg,
            double timeoutSec
    ) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        drive.cancelDistanceMove();
        drive.beginForwardDistanceInches(inches);

        boolean done = false;

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double currentYaw = robotYaw.getYaw();

            done = drive.updateForwardDistanceHoldHeading(
                    inches,
                    targetDeg,
                    currentYaw,
                    basePct,
                    0.12
            );

            // --- debug telemetry (use one source of truth: avgTicks) ---
            double avgTicks = drive.getAverageEncoderTicks();
            double calcInches = avgTicks / 28.52;

            telemetry.addData("t", "%.2f", timer.seconds());
            telemetry.addData("done", done);
            telemetry.addData("avgTicks", "%.0f", avgTicks);
            telemetry.addData("calcIn", "%.2f", calcInches);
            telemetry.addData("yawErr", "%.1f", targetDeg - currentYaw);
            telemetry.update();

            if (done) {
                drive.stopDriveVelocity();   // << use velocity stop
                break;
            }

            idle();
        }

        boolean timedOut = timer.seconds() >= timeoutSec;

        // final telemetry
        double avgTicks = drive.getAverageEncoderTicks();
        double calcInches = avgTicks / 28.52;
        telemetry.addData("EXIT", timedOut ? "TIMEOUT" : "DONE");
        telemetry.addData("finalTicks", "%.0f", avgTicks);
        telemetry.addData("finalCalcIn", "%.2f", calcInches);
        telemetry.update();

        // ✅ Only do anything extra if timed out (optional)
        if (opModeIsActive() && timedOut) {
            drive.stopDriveVelocity();
        }

        drive.cancelDistanceMove();
        drive.runUsingEncoders();
    }


    protected void strafeInchesVel(double inches, double basePct, double targetDeg,
                                   double timeoutSec, boolean right) {

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        drive.cancelDistanceMove();
        drive.beginStrafeDistanceInches(inches);

        boolean done = false;

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double currentYaw = robotYaw.getYaw();

            done = drive.updateStrafeDistanceHoldHeading(
                    inches, right,
                    targetDeg, currentYaw,
                    basePct,
                    0.12
            );

            double yawErr = targetDeg - currentYaw;
            telemetry.addData("Mode", "Strafe OptionA " + (right ? "Right" : "Left"));
            telemetry.addData("YawErr", "%.1f", yawErr);
            telemetry.addData("Done", done);
            telemetry.addData("t", "%.2f", timer.seconds());
            telemetry.update();

            if (done) {
                drive.stopDriveVelocity();   // ✅ stop NOW
                break;
            }
            idle();
        }

        boolean timedOut = timer.seconds() >= timeoutSec;

        // ✅ Optional: only do extra stop logic on timeout
        if (opModeIsActive() && timedOut) {
            drive.stopDriveVelocity();
        }

        drive.cancelDistanceMove();
        drive.runUsingEncoders();
    }




}
