package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PushBar;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherControl;

import java.util.ArrayList;
import java.util.List;

@TeleOp()
public class RobotOrientDrive extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    PushBar bar = new PushBar();
    IntakeControl intake = new IntakeControl();
    LauncherControl launch = new LauncherControl();
    List<Double> barPositions = new ArrayList<>(2);
    boolean launching = false;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    double launchSpeed;
    double shooterAdjustStep = 500;
    double shooterTargetRPM = 7000;
    @Override
    public  void init() {
        drive.init(hardwareMap);
        bar.init(hardwareMap);
        intake.init(hardwareMap);
        launch.init(hardwareMap);
    }

    @Override
    public void start(){
        bar.pushBall(0.7, 1.0);
        barPositions = bar.getBarPosition();
        telemetry.addData("Left Bar, Right Bar: ", barPositions);
        telemetry.update();
    }

    @Override
    public void loop() {
        // use gamepad sticks to control driving
        double forward = gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        drive.drive(forward, right, rotate);

        // press button to swing bars, release to return
        if(gamepad1.right_bumper) {
            bar.pushBall(0.55, 0.2);
            telemetry.addData("after push", bar.getBarPosition());
            telemetry.update();
        }
        else {
            bar.release(0.65, 1.0);
            telemetry.addData("after release", bar.getBarPosition());
            telemetry.update();
        }

        // activate intake
        if(gamepad1.left_bumper) {
            intake.setIntakePower(-1.0);
        }
        else {
            intake.setIntakePower(gamepad1.left_trigger);
        }


        // shoot

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        if(dpadUp && !lastDpadUp){
            shooterTargetRPM = launch.launchRPMAdjust(shooterAdjustStep, shooterTargetRPM);
        }
        if(dpadDown && !lastDpadDown){
            shooterTargetRPM = launch.launchRPMAdjust(-shooterAdjustStep, shooterTargetRPM);
        }
        lastDpadDown = dpadDown;
        lastDpadUp = dpadUp;

        if(gamepad1.a && !launching) {
            launch.launchBall(shooterTargetRPM);
            launchSpeed = launch.getLaunchRPM();
            telemetry.addData("Target Shooter RPM: ", shooterTargetRPM);
            telemetry.addData("Current Shooter RPM: ", launchSpeed);
            telemetry.update();
            launching = true;
        }

        if(gamepad1.y && launching) {
            launch.launchStop(0);
            launchSpeed = launch.getLaunchRPM();
            telemetry.addData("Launch stops at power: ", "0");
            telemetry.addData("Launch RPM: ", launchSpeed);
            telemetry.update();
            launching = false;
        }
    }
}