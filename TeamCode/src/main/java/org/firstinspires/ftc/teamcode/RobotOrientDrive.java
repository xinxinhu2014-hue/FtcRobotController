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

    private double wheelTargetRpm;
    double launchRpm, launchRpmError, BOOST_ON = 250, BOOST_OFF = 125, rpmTolerance = 75;
    int shootingType = 0, settleCount = 0;

    private final int SETTLE_REQUIREMENT = 1;


    ElapsedTime intakeTimer = new ElapsedTime(), shootPauseTimer = new ElapsedTime(), rollUpTimer = new ElapsedTime(),
            launchingTimer = new ElapsedTime(), wheelRecoveryTimer = new ElapsedTime(), wallLoosenTimer = new ElapsedTime();
    boolean launching = false, lastDpadUp = false, lastDpadDown = false, boosting = false, velocityControl = false,
            isTimedIntakeing = false, isBallFired = false, isMultiBallShooting = false, singleShotMode = false, lastWheelReady = false,
            wheelReadyEdge = false, launchInRange = false, firstTimeInRange = false, isWheelReady = false, isRollerUp = false, isWallLoosen = false,
            isSingleBallShooting = false, isBallReady = false;
    int ballCount = 0, isHere = 0;
    double inRangeTime = 0.0, outRangeTime = 0.0, firstInRangeTime = 0.0,
            rollerUpWaitTime = 0.0, shootPauseWaitTime = 0.0, wheelRecoverWaitTime = 0.0, wallLoosenWaitTime = 0.0;
    double leftGateClosePosition = 0.22, rightGateClosePosition = 0.35,
            leftGateOpenPosition = 0.6, rightGateOpenPosition = 0.5,
            leftWallLoosePosition = 0.74, rightWallLoosePosition = 0.7, leftWallTightPosition = 0.14, rightWallTightPosition = 0.18;
    double RpmAdjustBallTwo = 0, RpmAdjustBallThree = 0, wheelRecoverTimeLimitBallTwo = 1.5, wheelRecoverTimeLimitBallThree = 3.0;

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
        wall.loosenWall(leftWallLoosePosition, rightWallLoosePosition);
        intake.setIntakePower(0.0);
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
        if (!isTimedIntakeing && !isMultiBallShooting) {
            intake.setIntakePower(Math.min(1.0,gamepad1.left_trigger));
            if (gamepad1.left_trigger > 0) {
                wall.tightenWall(leftWallTightPosition, rightWallTightPosition); // test out position for tightening
            } else {
                wall.loosenWall(leftWallLoosePosition, rightWallLoosePosition); // test out position for loosening
            }
        }

        // left bumper: run intake for 0.2 second.
        if (gamepad1.leftBumperWasPressed() && !isTimedIntakeing) {
            isTimedIntakeing = true;
            intakeTimer.reset();
            //wall.tightenWall(0.16, 0.2); // test out position for tightening
            intake.setIntakePower(0.9);
        }

        if (isTimedIntakeing && intakeTimer.seconds() >= 0.2) {
            intake.setIntakePower(0.0);             // Stop motor
            isTimedIntakeing = false;
        }










        // Launching flywheels
        // target RPM adjustment: dpad up, dpad down
        // start and stop launching motor: a (start), y (stop)



        // launching flywheels: dpad up - nudge up target RPM every press by a fixed amount
        boolean dpadUp = gamepad1.dpad_up;
        double wheelRpmAdjustment = 100.0;
        if (dpadUp && !lastDpadUp && launching) {
            wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, wheelRpmAdjustment);
            rpmTolerance = 0.05 * wheelTargetRpm;
            launch.useVelocityControl(wheelTargetRpm); // << apply new target while running
        }
            lastDpadUp = dpadUp;

        // launching flywheels: dpad down - nudge down target RPM every press by a fixed amount
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !lastDpadDown && launching) {
            wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, -wheelRpmAdjustment);
            rpmTolerance = 0.05 * wheelTargetRpm;
            launch.useVelocityControl(wheelTargetRpm); // << apply new target while running
        }
        lastDpadDown = dpadDown;

        // launching flywheels: a - start launching wheel at high speed
        if (gamepad1.a && !launching) {
            wheelTargetRpm = 4500.0; // at small launching zone
            rpmTolerance = 0.05 * wheelTargetRpm;
            launching = true;
            isBallReady = true;
            settleCount = 0;
            shootingType = 3;
            launchingTimer.reset();
            outRangeTime = launchingTimer.seconds();
            launchInRange = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            inRangeTime = 0.0;
            RpmAdjustBallTwo = 600;
            RpmAdjustBallThree = 350;
            wheelRecoverTimeLimitBallTwo = 2.5;
            wheelRecoverTimeLimitBallThree = 2.5;
        }

        // launching flywheels: b - start launching wheel at medium speed
        if (gamepad1.b && !launching) {
            wheelTargetRpm = 3700.0; // about 50" shooting distance
            rpmTolerance = 0.05 * wheelTargetRpm;
            launching = true;
            isBallReady = true;
            settleCount = 0;
            shootingType = 2;
            launchingTimer.reset();
            outRangeTime = launchingTimer.seconds();
            launchInRange = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            inRangeTime = 0.0;
            RpmAdjustBallTwo = 300;
            RpmAdjustBallThree = 0;
            wheelRecoverTimeLimitBallTwo = 1.5;
            wheelRecoverTimeLimitBallThree = 1.0;
        }

        // launching flywheels: x - start launching wheel at low speed
        if (gamepad1.x && !launching) {
            launching = true;
            isBallReady = true;
            settleCount = 0;
            wheelTargetRpm = 3300.0; // about 15" shooting distance
            rpmTolerance = 0.05 * wheelTargetRpm;
            shootingType = 1;
            launchingTimer.reset();
            outRangeTime = launchingTimer.seconds();
            launchInRange = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            inRangeTime = 0.0;
            RpmAdjustBallTwo = 100;
            RpmAdjustBallThree = 0;
            wheelRecoverTimeLimitBallTwo = 1.5;
            wheelRecoverTimeLimitBallThree = 1.0;
        }

        if (launching) {
            launchRpm = launch.currentWheelRpm();
            launchRpmError = wheelTargetRpm - launchRpm;
            if (launchRpmError > BOOST_ON) {
                // boost phase
                if (!boosting) {
                    launch.boostLaunch(1.0);
                    boosting = true;
                    velocityControl = false;
                }
            } else if (boosting && launchRpmError < BOOST_OFF) {
                boosting = false;
            }

            if (!boosting && !velocityControl) {
                launch.useVelocityControl(wheelTargetRpm);
                velocityControl = true;
            }

            if (velocityControl && Math.abs(launchRpmError) <= rpmTolerance) {
                settleCount++;
            } else {
                settleCount = 0;
            }


            isWheelReady = (settleCount >= SETTLE_REQUIREMENT);
            wheelReadyEdge = isWheelReady && !lastWheelReady;
            lastWheelReady = isWheelReady;
        }

        // launching flywheels: y - stop launching wheel
        if (gamepad1.y && launching) {
            launch.stopLaunch();
            launching = false;
            ballCount = 0;
            wall.loosenWall(leftWallLoosePosition, rightWallLoosePosition);
            isBallFired = false;
            isMultiBallShooting = false;
            gate.closeDoor(leftGateClosePosition, rightGateClosePosition);
            inRangeTime = 0.0;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            shootingType = 0;
            RpmAdjustBallTwo = 0;
            RpmAdjustBallThree = 0;
            wheelRecoverTimeLimitBallTwo = 1.5;
            wheelRecoverTimeLimitBallThree = 1.5;
            boosting = false;
            velocityControl = false;
            isWheelReady = false;
            lastWheelReady = false;
            wheelReadyEdge = false;
            isBallReady = false;
        }








        // 3 balls shooting
        // use Right Bumper
        if (gamepad1.right_bumper && isWheelReady && !isBallFired && !isMultiBallShooting && !isSingleBallShooting && isBallReady) {
            // Mark 3-ball shooting process start
            isMultiBallShooting = true;
            isBallReady = false;
            // Wall tightens to give shooter enough space
            wall.tightenWall(leftWallTightPosition, rightWallTightPosition);
            // Gate opens and 1st ball out
            gate.openDoor(leftGateOpenPosition, rightGateOpenPosition);
            ballCount = ballCount + 1;
            isBallFired = true;
            shootPauseTimer.reset();
            shootPauseWaitTime = 0.3;
            isWheelReady = false;
            settleCount = 0;
        }

        if (isBallFired && !isBallReady && shootPauseTimer.seconds() >= shootPauseWaitTime && ballCount == 1) {
            isBallFired = false;
            isBallReady = true;
            // Adjust wheel RPM for the remaining balls
            //wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, RpmAdjustBallTwo); // launch RPM adjustment for 2nd ball
            //rpmTolerance = 0.05 * wheelTargetRpm;
            wheelRecoverWaitTime = wheelRecoverTimeLimitBallTwo;
            //launch.useVelocityControl(wheelTargetRpm);
            launching = true;
            wheelRecoveryTimer.reset(); // launch timeout limit if RPM tolerance cannot be reached
            //isWheelReady = true;
        }

        if (isBallFired && !isBallReady && shootPauseTimer.seconds() >= shootPauseWaitTime && ballCount == 2) {
            isBallFired = false;
            wall.loosenWall(leftWallLoosePosition, rightWallLoosePosition);
            wallLoosenWaitTime = 0.2;
            isWallLoosen = true;
            wallLoosenTimer.reset();
        }

        if (isWallLoosen && !isBallReady && wallLoosenTimer.seconds() > wallLoosenWaitTime) {
            isBallReady = true;
            wall.tightenWall(leftWallTightPosition, rightWallTightPosition);
            // Adjust wheel RPM for the remaining balls
            //wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, RpmAdjustBallThree); // launch RPM adjustment for 3rd ball
            //rpmTolerance = 0.05 * wheelTargetRpm;
            wheelRecoverWaitTime = wheelRecoverTimeLimitBallThree;
            //launch.useVelocityControl(wheelTargetRpm);
            launching = true;
            wheelRecoveryTimer.reset(); // launch timeout limit if RPM tolerance cannot be reached
            //isWheelReady = true;
            isWallLoosen = false;
        }


        if (isBallReady && isMultiBallShooting && (wheelReadyEdge || wheelRecoveryTimer.seconds() >= wheelRecoverWaitTime) && ballCount < 3) {
            isBallReady = false;
            isWheelReady = false;
            settleCount = 0;
            // Move up the remaining balls
            intake.setIntakePower(1.0);
            if (ballCount == 1) {
                rollerUpWaitTime = 0.2;
            } else {
                rollerUpWaitTime = 1.5;
            }
            isRollerUp = true;
            rollUpTimer.reset();
        }

        if (isRollerUp && rollUpTimer.seconds() >= rollerUpWaitTime && ballCount < 3) {
            isHere++;
            isRollerUp = false;
            intake.setIntakePower(0.0);
            ballCount = ballCount + 1;
            shootPauseWaitTime = 0.3;
            isBallFired = true;
            shootPauseTimer.reset();
        }

        if (ballCount == 3 && rollUpTimer.seconds() > rollerUpWaitTime) { // 3-ball shooting process ends, reset everything
            ballCount = 0;
            isBallFired = false;
            isMultiBallShooting = false;
            intake.setIntakePower(0.0);
            gate.closeDoor(leftGateClosePosition, rightGateClosePosition);
            wall.loosenWall(leftWallLoosePosition, rightWallLoosePosition);
            launch.stopLaunch();
            launching = false;
            boosting = false;
            velocityControl = false;
            isWheelReady = false;
            lastWheelReady = false;
            wheelReadyEdge = false;
            isBallReady = false;
        }







        // single ball shooting
        // use Right Trigger
        if (!isSingleBallShooting && !isMultiBallShooting && launching) {
            if (gamepad1.right_trigger > 0.7) {
                isSingleBallShooting = true;
                wall.tightenWall(leftWallTightPosition, rightWallTightPosition);
                intake.setIntakePower(1.0);
            }
        }

        if (isSingleBallShooting && !isMultiBallShooting && launching) {
            if (isWheelReady) {
                gate.openDoor(leftGateOpenPosition, rightGateOpenPosition);
            }
            if (gamepad1.right_trigger < 0.2) {
                gate.closeDoor(leftGateClosePosition, rightGateClosePosition);
                wall.loosenWall(leftWallLoosePosition, rightWallLoosePosition);
                intake.setIntakePower(0.0);
                launch.stopLaunch();
                launching = false;
                boosting = false;
                velocityControl = false;
                isWheelReady = false;
                lastWheelReady = false;
                wheelReadyEdge = false;
                isSingleBallShooting = false;
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
            /*
            if (launchRpmError < 0) {
                telemetry.addData("BELOW target by: ", Math.abs(launchRpmError));
            }
            if (launchRpmError > 0) {
                telemetry.addData("ABOVE target by: ", Math.abs(launchRpmError));
            }
             */
            if (Math.abs(launchRpmError) <= rpmTolerance) {
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
            telemetry.addData("Ball fired:  ", ballCount);

            //
            //telemetry.addData("First time in range in seconds of ", firstInRangeTime);
            //telemetry.addData("In range in seconds of ", inRangeTime);

        }
        telemetry.addData("ball ready ", isBallReady);
        telemetry.addData("wheel edge ready ", wheelReadyEdge);
        telemetry.addData("wheel ready ", isWheelReady);
        telemetry.addData("last wheel ready ", lastWheelReady);
        telemetry.addData("multi shoot", isMultiBallShooting);
        telemetry.addData("single shoot", isSingleBallShooting);
        telemetry.addData("trigger?", gamepad1.right_bumper);
        telemetry.addData("settle ", settleCount);
        telemetry.update();
    }
}