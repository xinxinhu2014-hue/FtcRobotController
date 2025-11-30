package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ReleaseDoors;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherControl;

@TeleOp()
public class RobotOrientDrive extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    ReleaseDoors bar = new ReleaseDoors();
    IntakeControl intake = new IntakeControl();
    LauncherControl launch = new LauncherControl();
    private boolean launching = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private double wheelTargetRpm;
    private final double wheelRpmAdjustment = 100.0;
    double launchRpm, launchRpmError;
    int shootingType = 0;
    ElapsedTime timer = new ElapsedTime();
    boolean inTimedIntakeing = false;


    @Override
    public void init() {
        drive.init(hardwareMap);
        bar.init(hardwareMap);
        intake.init(hardwareMap);
        launch.init(hardwareMap);
        timer.reset();
    }

    @Override
    public void start() {
        bar.closeDoor(0.0, 0.1);
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

        // press right bumper to swing bars, closeDoor to reset
        if (gamepad1.right_bumper) {
            bar.openDoor(0.6, 0.5);
            //telemetry.addData("after push", bar.getBarPosition());
        } else {
            bar.closeDoor(0.0, 0.1);
            //telemetry.addData("after closeDoor", bar.getBarPosition());
        }

        // intake
        // left trigger: take in first 2 balls
        if (!inTimedIntakeing) {
            intake.setIntakePower(gamepad1.left_trigger);
        }

        // left bumper: run intake for 1 second.
        if (gamepad1.leftBumperWasPressed() && !inTimedIntakeing) {
            inTimedIntakeing = true;
            timer.reset();
            intake.setIntakePower(0.9);
        }

        if (inTimedIntakeing && timer.seconds() >= 0.2) {
            intake.setIntakePower(0.0);          // Stop motor
            inTimedIntakeing = false;
        }




        // shoot
        // target RPM adjustment: dpad up, dpad down
        // start and stop launching motor: a (start), y (stop)

        // shoot: dpad up - nudge up target RPM every press by a fixed amount
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp && launching) {
            wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, wheelRpmAdjustment);
            launch.startLaunch(wheelTargetRpm); // << apply new target while running
        }
            lastDpadUp = dpadUp;

        // shoot: dpad down - nudge down target RPM every press by a fixed amount
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !lastDpadDown && launching) {
            wheelTargetRpm = launch.adjustLaunchRpm(wheelTargetRpm, -wheelRpmAdjustment);
            launch.startLaunch(wheelTargetRpm); // << apply new target while running
        }
        lastDpadDown = dpadDown;

        // shoot: a - start launching wheel at high speed
        if (gamepad1.a && !launching) {
            wheelTargetRpm = 4071.0; // at small launching zone
            launch.startLaunch(wheelTargetRpm);
            launching = true;
            shootingType = 3;
        }

        // shoot: b - start launching wheel at medium speed
        if (gamepad1.b && !launching) {
            wheelTargetRpm = 3400.0; // about 50" shooting distance
            launch.startLaunch(wheelTargetRpm);
            launching = true;
            shootingType = 2;
        }

        // shoot: x - start launching wheel at low speed
        if (gamepad1.x && !launching) {
            wheelTargetRpm = 3000.0; // about 15" shooting distance
            launch.startLaunch(wheelTargetRpm);
            launching = true;
            shootingType = 1;
        }

        // shoot: y - stop launching wheel
        if (gamepad1.y && launching) {
            launch.stopLaunch();
            launching = false;
            shootingType = 0;
        }

        launchRpm = launch.currentWheelRpm();
        launchRpmError = launchRpm - wheelTargetRpm;
        telemetry.addData("Shooter", launching ? "ON" : "OFF");
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
                telemetry.addLine("RPM is in the range, recommend fire!");
            }
        }
        telemetry.update();
    }
}