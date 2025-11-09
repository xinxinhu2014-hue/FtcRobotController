package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LauncherControl {

    private DcMotor launchMotor;
    private double ticksPerRotation;
    ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hardwareMap){
        launchMotor = hardwareMap.get(DcMotor.class, "shooter");
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ticksPerRotation = 28;
    }

    public void launchBall(double power) {
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setPower(power);
    }

    public double getLaunchRPM() {
        return launchMotor.getCurrentPosition() / ticksPerRotation * 60;
    }
}
