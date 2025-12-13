package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ReleaseDoors;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherControl;
import org.firstinspires.ftc.teamcode.mechanisms.WallControl;

@TeleOp()
public class RobotOrientDrive extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    ReleaseDoors gate = new ReleaseDoors();
    IntakeControl intake = new IntakeControl();
    LauncherControl launch = new LauncherControl();
    WallControl wall = new WallControl();
    private boolean launching = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private double wheelTargetRpm;
    private final double wheelRpmAdjustment = 100.0;
    double launchRpm, launchRpmError;
    int shootingType = 0;
    boolean lastLB = false;
    ElapsedTime doubleTimer = new ElapsedTime();
    final double DOUBLE_PRESS_WINDOW = 0.30;  // time allowed between presses (seconds)

    ElapsedTime intakeTimer = new ElapsedTime(), rollDownTimer = new ElapsedTime(), rollUpTimer = new ElapsedTime(),
            launchingTimer = new ElapsedTime(), wheelRecoveryTimer = new ElapsedTime(), doorClosedTimer = new ElapsedTime();
    boolean inTimedIntakeing = false, isBallFired = false, isRolling = false, shootingNotFinish = false,
            launchInRange = false, firstTimeInRange = false, isWheelRecovered = false, isDoorClosed = false;
    int ballCount = 0;
    double boostingTime, inRangeTime = 0.0, outRangeTime = 0.0, firstInRangeTime = 0.0,
            rollerUpWaitTime = 0.0, rollDownWaitTime = 0.0, wheelRecoverWaitTime = 0.0, doorClosedWaitTime = 0.0;
    double leftGateClosePosition = 0.21, rightGateClosePosition = 0.31,
            leftGateOpenPosition = 0.6, rightGateOpenPosition = 0.5;
    double RpmAdjustBallTwo = 0, RpmAdjustBallThree = 0, wheelRecoverTimeLimitBallTwo = 1.5, wheelRecoverTimeLimitBallThree = 1.5;

    @Override
    public void init() {
        drive.init(hardwareMap);
        gate.init(hardwareMap);
        intake.init(hardwareMap);
        launch.init(hardwareMap);
        wall.init(hardwareMap);
        intakeTimer.reset();
    }

    @Override
    public void start() {
        gate.closeDoor(leftGateClosePosition, rightGateClosePosition);
        wall.loosenWall(0.74, 0.7);
    }

    private double dead(double v){
        return Math.abs(v) < 0.05 ? 0.0 : v;
    }


    @Override
    public void loop() {
        // use gamepad sticks to control driving
        double forward = gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        forward = dead(forward);
        right = dead(right);
        rotate = dead(rotate);
        drive.drive(forward, right, rotate);


        // intake
        // left trigger: take in first 2 balls from field
        // right trigger: take balls from human player from top
        if (!inTimedIntakeing && !shootingNotFinish) {
            intake.setIntakePower(Math.min(0.6,gamepad1.left_trigger));
            if (gamepad1.left_trigger > 0) {
                wall.tightenWall(0.16, 0.2); // test out position for tightening
            } else {
                wall.loosenWall(0.74, 0.7); // test out position for loosening
            }
        }

        // left bumper: run intake for 0.2 second.
        if (gamepad1.leftBumperWasPressed() && !inTimedIntakeing) {
            inTimedIntakeing = true;
            intakeTimer.reset();
            //wall.tightenWall(0.16, 0.2); // test out position for tightening
            intake.setIntakePower(0.9);
        }

        if (inTimedIntakeing && intakeTimer.seconds() >= 0.2) {
            intake.setIntakePower(0.0);             // Stop motor
            inTimedIntakeing = false;
        }







        // Launching flywheels
        // target RPM adjustment: dpad up, dpad down
        // start and stop launching motor: a (start), y (stop)

        double RPM_TOLERANCE = 75;

        // launching flywheels: dpad up - nudge up target RPM every press by a fixed amount
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp && launching) {
            wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, wheelRpmAdjustment);
            RPM_TOLERANCE = 0.05 * wheelTargetRpm;
            launch.useVelocityControl(wheelTargetRpm); // << apply new target while running
        }
            lastDpadUp = dpadUp;

        // launching flywheels: dpad down - nudge down target RPM every press by a fixed amount
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !lastDpadDown && launching) {
            wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, -wheelRpmAdjustment);
            RPM_TOLERANCE = 0.05 * wheelTargetRpm;
            launch.useVelocityControl(wheelTargetRpm); // << apply new target while running
        }
        lastDpadDown = dpadDown;

        // launching flywheels: a - start launching wheel at high speed
        if (gamepad1.a && !launching) {
            wheelTargetRpm = 4400.0; // at small launching zone
            RPM_TOLERANCE = 0.05 * wheelTargetRpm;
            launching = true;
            shootingType = 3;
            launchingTimer.reset();
            outRangeTime = launchingTimer.seconds();
            launchInRange = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            inRangeTime = 0.0;
            boostingTime = 1.4;
            RpmAdjustBallTwo = 600;
            RpmAdjustBallThree = 350;
            wheelRecoverTimeLimitBallTwo = 2.5;
            wheelRecoverTimeLimitBallThree = 2.5;
        }

        // launching flywheels: b - start launching wheel at medium speed
        if (gamepad1.b && !launching) {
            wheelTargetRpm = 3600.0; // about 50" shooting distance
            RPM_TOLERANCE = 0.05 * wheelTargetRpm;
            launching = true;
            shootingType = 2;
            launchingTimer.reset();
            outRangeTime = launchingTimer.seconds();
            launchInRange = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            inRangeTime = 0.0;
            boostingTime = 0.9;
            RpmAdjustBallTwo = 300;
            RpmAdjustBallThree = 0;
            wheelRecoverTimeLimitBallTwo = 1.5;
            wheelRecoverTimeLimitBallThree = 1.0;
        }

        // launching flywheels: x - start launching wheel at low speed
        if (gamepad1.x && !launching) {
            launching = true;
            wheelTargetRpm = 3000.0; // about 15" shooting distance
            RPM_TOLERANCE = 0.05 * wheelTargetRpm;
            shootingType = 1;
            launchingTimer.reset();
            outRangeTime = launchingTimer.seconds();
            launchInRange = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            inRangeTime = 0.0;
            boostingTime =0.7;
            RpmAdjustBallTwo = 200;
            RpmAdjustBallThree = 0;
            wheelRecoverTimeLimitBallTwo = 1.5;
            wheelRecoverTimeLimitBallThree = 1.0;
        }

        if (launching) {
            launchRpm = launch.currentWheelRpm();
            launchRpmError = wheelTargetRpm - launchRpm;
            if (launchRpmError > 200) {
                // boost phase
                launch.boostLaunch(1.0);
            } else {
                // PIDF hold phase
                launch.useVelocityControl(wheelTargetRpm);
            }
        }

        // launching flywheels: y - stop launching wheel
        if (gamepad1.y && launching) {
            launch.stopLaunch();
            ballCount = 0;
            wall.loosenWall(0.74, 0.7);
            isBallFired = false;
            shootingNotFinish = false;
            gate.closeDoor(leftGateClosePosition, rightGateClosePosition);
            inRangeTime = 0.0;
            launching = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            shootingType = 0;
            RpmAdjustBallTwo = 0;
            RpmAdjustBallThree = 0;
            wheelRecoverTimeLimitBallTwo = 1.5;
            wheelRecoverTimeLimitBallThree = 1.5;
        }








        // 3 balls shooting
        if (gamepad1.rightBumperWasPressed() && !isBallFired && !shootingNotFinish) {
            // Mark 3-ball shooting process start
            shootingNotFinish = true;
            // Gate opens and top ball out
            gate.openDoor(leftGateOpenPosition, rightGateOpenPosition);
            ballCount = ballCount + 1;
            isBallFired = true;
            // Wall tightens and below balls move down
            wall.tightenWall(0.16, 0.20);
            //intake.setIntakePower(0.0); // below balls start moving down
            rollDownTimer.reset(); // first ball, no rolling down. It's time pausing for close the door.
            rollDownWaitTime = 0.1;
        }

        if (isBallFired && rollDownTimer.seconds() >= rollDownWaitTime && ballCount < 3) {
            isBallFired = false;
            // Stop moving down and stay
            intake.setIntakePower(0.0);
            if (ballCount == 1) {
                gate.closeDoor(leftGateClosePosition, rightGateClosePosition);
            }
            isDoorClosed = true;
            doorClosedTimer.reset(); // It's time pausing for the door remained closing. Only work for first ball. After shooting 2nd ball, the door keeps open.
            doorClosedWaitTime = 0.1;

            if(ballCount == 1) {
                wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, RpmAdjustBallTwo); // launch RPM adjustment for 2nd ball
                RPM_TOLERANCE = 0.05 * wheelTargetRpm;
                wheelRecoverWaitTime = wheelRecoverTimeLimitBallTwo;
            } else {
                wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, RpmAdjustBallThree); // launch RPM adjustment for 3rd ball
                RPM_TOLERANCE = 0.05 * wheelTargetRpm;
                wheelRecoverWaitTime = wheelRecoverTimeLimitBallThree;
            }
            launch.useVelocityControl(wheelTargetRpm);
            launching = true;
            wheelRecoveryTimer.reset(); // launch timeout limit if RPM tolerance cannot be reached
        }

        if (isDoorClosed && doorClosedTimer.seconds() >= doorClosedWaitTime && ballCount < 3) {
            isDoorClosed = false;

            if (ballCount == 1) { // rolling a little to nudge 2nd ball behind the closed gate
                isRolling = true;
                intake.setIntakePower(0.9);
                rollUpTimer.reset();
                rollerUpWaitTime = 0.1;
            } else {
                isWheelRecovered = true; // 3rd ball in the low level of the tunnel, waiting for signal of launch ready to roll up
            }
        }

        if (isRolling && rollUpTimer.seconds() >= rollerUpWaitTime && ballCount < 3) {
            isRolling = false; // only 2nd ball enters this section, ball ready behind the closed gate
            intake.setIntakePower(0.0);
            isWheelRecovered = true;
        }
        launchRpm = launch.currentWheelRpm();
        launchRpmError = launchRpm - wheelTargetRpm;

        if (isWheelRecovered && (Math.abs(launchRpmError) <= RPM_TOLERANCE || wheelRecoveryTimer.seconds() >= wheelRecoverWaitTime) && ballCount < 3) {
            isWheelRecovered = false;
            if (ballCount == 1) { // gate open to let 2nd ball out, 3rd ball moving down the tunnel
                gate.openDoor(leftGateOpenPosition, rightGateOpenPosition);
                intake.setIntakePower(-0.6);
                rollDownTimer.reset();
                rollDownWaitTime = 0.3;
            } else {
                intake.setIntakePower(0.9); // 3rd ball get the signal of launch ready, start moving up the tunnel
                rollUpTimer.reset();
                rollerUpWaitTime = 1.5;
            }
            ballCount = ballCount + 1;
            isBallFired = true;
        }


        if (ballCount == 3 && rollUpTimer.seconds() > rollerUpWaitTime) { // 3-ball shooting process ends, reset everything
            ballCount = 0;
            wall.loosenWall(0.74, 0.7);
            isBallFired = false;
            shootingNotFinish = false;
            gate.closeDoor(leftGateClosePosition, rightGateClosePosition);
            launch.stopLaunch();
            launching = false;
        }

        // single ball shooting
        if (!shootingNotFinish) {
            if (gamepad1.right_trigger > 0) {
                gate.openDoor(leftGateOpenPosition, rightGateOpenPosition);
            } else {
                gate.closeDoor(leftGateClosePosition, rightGateClosePosition);
            }
        }







        telemetry.addData("Shooter", launching ? "ON" : "OFF");
        telemetry.addData("Target RPM", wheelTargetRpm);
        telemetry.addData("Current RPM", launchRpm);
        if(launching) {
            if(shootingType == 3){
                telemetry.addLine("Far shot");
            }
            if(shootingType == 2){
                telemetry.addLine("Middle shot");
            }
            if(shootingType == 1){
                telemetry.addLine("Close shot");
            }
            if (launchRpmError < 0) {
                telemetry.addData("BELOW target by: ", Math.abs(launchRpmError));
            }
            if (launchRpmError > 0) {
                telemetry.addData("ABOVE target by: ", Math.abs(launchRpmError));
            }
            if (Math.abs(launchRpmError) <= 75) {
                if(!launchInRange) {
                    inRangeTime = launchingTimer.seconds() - outRangeTime;
                    launchInRange = true;
                    if (!firstTimeInRange){
                        firstInRangeTime = inRangeTime;
                        firstTimeInRange = true;
                    }
                }
                telemetry.addLine("RPM is in the range, recommend fire!");
            } else {
                if (launchInRange) {
                    outRangeTime = launchingTimer.seconds();
                    launchInRange = false;
                }
            }
            telemetry.addData("Launcher is running for ", launchingTimer.seconds());
            telemetry.addData("First time in range in seconds of ", firstInRangeTime);
            telemetry.addData("In range in seconds of ", inRangeTime);
        }
        telemetry.update();
    }
}