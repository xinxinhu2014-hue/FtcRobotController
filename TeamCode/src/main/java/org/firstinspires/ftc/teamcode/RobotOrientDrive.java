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
    ElapsedTime intakeTimer = new ElapsedTime(), rollDownTimer = new ElapsedTime(), rollUpTimer = new ElapsedTime(),
            launchingTimer = new ElapsedTime(), wheelRecoveryTimer = new ElapsedTime();
    boolean inTimedIntakeing = false, isShooting = false, isRolling = false, shootingNotFinish = false,
            launchInRange = false, firstTimeInRange = false, isWheelRecovered = false;
    int ballCount = 0;
    double inRangeTime = 0.0, outRangeTime = 0.0, firstInRangeTime = 0.0,
            rollerUpWaitTime = 0.0, rollDownWaitTime = 0.0;
    double boostingTime;

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
        gate.closeDoor(0.1, 0.2);
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
            intake.setIntakePower(gamepad1.left_trigger);
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

        // launching flywheels: dpad up - nudge up target RPM every press by a fixed amount
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp && launching) {
            wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, wheelRpmAdjustment);
            launch.useVelocityControl(wheelTargetRpm); // << apply new target while running
        }
            lastDpadUp = dpadUp;

        // launching flywheels: dpad down - nudge down target RPM every press by a fixed amount
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !lastDpadDown && launching) {
            wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, -wheelRpmAdjustment);
            launch.useVelocityControl(wheelTargetRpm); // << apply new target while running
        }
        lastDpadDown = dpadDown;

        // launching flywheels: a - start launching wheel at high speed
        if (gamepad1.a && !launching) {
            wheelTargetRpm = 4071.0; // at small launching zone
            launching = true;
            shootingType = 3;
            launchingTimer.reset();
            outRangeTime = launchingTimer.seconds();
            launchInRange = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            inRangeTime = 0.0;
            boostingTime = 1.1;
        }

        // launching flywheels: b - start launching wheel at medium speed
        if (gamepad1.b && !launching) {
            wheelTargetRpm = 3400.0; // about 50" shooting distance
            launching = true;
            shootingType = 2;
            launchingTimer.reset();
            outRangeTime = launchingTimer.seconds();
            launchInRange = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            inRangeTime = 0.0;
            boostingTime = 0.9;
        }

        // launching flywheels: x - start launching wheel at low speed
        if (gamepad1.x && !launching) {
            launching = true;
            wheelTargetRpm = 3350.0; // about 15" shooting distance
            shootingType = 1;
            launchingTimer.reset();
            outRangeTime = launchingTimer.seconds();
            launchInRange = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            inRangeTime = 0.0;
            boostingTime =0.7;
        }

        if (launching) {
            if (launchingTimer.seconds() < boostingTime) {
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
            inRangeTime = 0.0;
            launching = false;
            firstTimeInRange = false;
            firstInRangeTime = 0.0;
            shootingType = 0;
        }

        // shooting
        if (gamepad1.rightBumperWasPressed() && !isShooting && !shootingNotFinish) {
            shootingNotFinish = true;
            isShooting = true;
            wall.tightenWall(0.16, 0.20);
            gate.openDoor(0.6, 0.5); // gate opens and top ball out
            intake.setIntakePower(-0.6); // below balls start moving down
            rollDownTimer.reset();
            ballCount = ballCount + 1;
            rollDownWaitTime = 0.4;
        }

        if (isShooting && rollDownTimer.seconds() >= rollDownWaitTime && ballCount < 3) {
            //gate.closeDoor(0.1, 0.2);
            intake.setIntakePower(0.0); // ball stop moving down and stay there
            wall.loosenWall(0.79, 0.75);
            isShooting = false;
            wheelRecoveryTimer.reset();
            isWheelRecovered = true;
            if(ballCount == 1) {
                wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, 150);
            } else {
                wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, -250);
            }
            launch.useVelocityControl(wheelTargetRpm);
        }

        if (isWheelRecovered && wheelRecoveryTimer.seconds() >= 1.0 && ballCount < 3) {
            wall.tightenWall(0.16, 0.2); // ball start moving up
            isRolling = true;
            isWheelRecovered = false;
            rollUpTimer.reset();
            intake.setIntakePower(0.7);
            if (ballCount == 1) {
                rollerUpWaitTime = 0.2;
            } else {
                rollerUpWaitTime = 0.4;
            }
        }


        if (isRolling && rollUpTimer.seconds() >= rollerUpWaitTime && ballCount < 3) {
            intake.setIntakePower(0.0); // ball stop moving up and top one gets out
            // wall.loosenWall(0.79, 0.75);
            isRolling = false;
            isShooting = true;
            //gate.openDoor(0.6, 0.5);
            if (ballCount == 2) {
                rollDownWaitTime = 0.4;
            } else {
                rollDownWaitTime = 0.4;
            }
            intake.setIntakePower(-0.5); // below balls start moving down
            rollDownTimer.reset();
            /*
            if (ballCount < 2) {
                wall.loosenWall(0.79, 0.75);
            } else {
                wall.tightenWall(0.11, 0.15);
            }
   */
            ballCount = ballCount + 1;
        }

        if (ballCount == 3 && rollDownTimer.seconds() > rollDownWaitTime) {
            ballCount = 0;
            wall.loosenWall(0.78, 0.7);
            isShooting = false;
            shootingNotFinish = false;
            gate.closeDoor(0.1, 0.2);
        }









        launchRpm = launch.currentWheelRpm();
        launchRpmError = launchRpm - wheelTargetRpm;
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
            if (Math.abs(launchRpmError) <= 50) {
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