package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoBase extends LinearOpMode {

    DcMotor fl, fr, bl, br;
    DcMotor launcher;

    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "frontleft");
        fr = hardwareMap.get(DcMotor.class, "frontright");
        bl = hardwareMap.get(DcMotor.class, "backleft");
        br = hardwareMap.get(DcMotor.class, "backright");

        launcher = hardwareMap.get(DcMotor.class, "shooter");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {fl, fr, bl, br};
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while(opModeIsActive()) {
            forward(0.3, 5000);
        }
    }

    public void drive(double powerFL, double powerFR, double powerBL, double powerBR, long ms) {
        fl.setPower(powerFL);
        fr.setPower(powerFR);
        bl.setPower(powerBL);
        br.setPower(powerBR);
        sleep(ms);
        stopDrive();
    }

    public void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void forward(double power, long ms) { drive(power, power, power, power, ms); }
    public void backward(double power, long ms) { drive(-power, -power, -power, -power, ms); }
    public void strafeLeft(double power, long ms) { drive(-power, power, power, -power, ms); }
    public void strafeRight(double power, long ms) { drive(power, -power, -power, power, ms); }
    public void turnRight(double power, long ms) { drive(power, -power, power, -power, ms); }
    public void turnLeft(double power, long ms) { drive(-power, power, -power, power, ms); }

    public void launch() {
        launcher.setPower(1);
        sleep(2000); // adjust for flywheel speed-up
        // fire servo here if you add one
        sleep(500);
        launcher.setPower(0);
    }


}
