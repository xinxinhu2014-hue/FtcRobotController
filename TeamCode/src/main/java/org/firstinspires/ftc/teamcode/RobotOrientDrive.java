package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PushBar;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherControl;

@TeleOp()
public class RobotOrientDrive extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    PushBar bar = new PushBar();
    IntakeControl intake = new IntakeControl();
    LauncherControl launch = new LauncherControl();
    private boolean launching = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private double wheelTargetRpm;
    private final double wheelRpmAdjustment = 100.0;
    double launchRpm;


    @Override
    public void init() {
        drive.init(hardwareMap);
        bar.init(hardwareMap);
        intake.init(hardwareMap);
        launch.init(hardwareMap);
    }

    @Override
    public void start() {
        bar.pushBall(0.65, 1.0);
    }

    private double dead(double v){
        return Math.abs(v) < 0.05 ? 0.0 : v;
    }


    @Override
    public void loop() {
        // use gamepad sticks to control driving
        double forward = gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        forward = dead(forward);
        right = dead(right);
        rotate = dead(rotate);
        drive.drive(forward, right, rotate);

        // press right bumper to swing bars, release to reset
        if (gamepad1.right_bumper) {
            bar.pushBall(0.55, 0.2);
            //telemetry.addData("after push", bar.getBarPosition());
        } else {
            bar.release(0.65, 1.0);
            //telemetry.addData("after release", bar.getBarPosition());
        }

        // intake
        // left trigger: take in balls with adjusted speed
        // left bumper: release jammed balls
        if (gamepad1.left_bumper) {
            intake.setIntakePower(-1.0);
        } else {
            intake.setIntakePower(gamepad1.left_trigger);
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
            wheelTargetRpm = 3800.0; // about 7 ft shooting distance
            launch.startLaunch(wheelTargetRpm);
            launching = true;
        }

        // shoot: b - start launching wheel at medium speed
        if (gamepad1.b && !launching) {
            wheelTargetRpm = 3400.0; // about 50" shooting distance
            launch.startLaunch(wheelTargetRpm);
            launching = true;
        }

        // shoot: x - start launching wheel at low speed
        if (gamepad1.x && !launching) {
            wheelTargetRpm = 2900.0; // about 15" shooting distance
            launch.startLaunch(wheelTargetRpm);
            launching = true;
        }

        // shoot: y - stop launching wheel
        if (gamepad1.y && launching) {
            launch.stopLaunch();
            launching = false;
        }

        launchRpm = launch.currentWheelRpm();
        telemetry.addData("Target wheel RPM", "%.0f", wheelTargetRpm);  // << no hard-code
        telemetry.addData("Actual wheel RPM", "%.0f", launchRpm);
        if (launchRpm <= wheelTargetRpm * 1.02 && launchRpm >= wheelTargetRpm * 0.99) {
            telemetry.addData("Shooter", "READY! READY!");
        }
        telemetry.update();
    }
}