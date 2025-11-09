package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LauncherControl {
    private static final double TICKS_PER_REV = 28.0;
    private static final double MOTOR_MAX_RPM = 6600;
    private static final double GEAR_RATIO = 50.0/30.0;
    private static final double WHEEL_MAX_RPM = MOTOR_MAX_RPM * GEAR_RATIO;
    private static final double WHEEL_MIN_RPM = 4500.0;
    private static final double kP = 12.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = kP / (MOTOR_MAX_RPM / 60 * TICKS_PER_REV);
    private DcMotorEx launchMotor;

    public void init(HardwareMap hardwareMap){
        launchMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        launchMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    private double rpmToTicksPerSec(double rpm) { return rpm * TICKS_PER_REV / 60.0; }
    private double ticksPerSecToRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }

    public void launchBall(double targetWheelRPM){
        launchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        launchMotor.setVelocity(rpmToTicksPerSec(targetWheelRPM / GEAR_RATIO));
    }
    public double launchRPMAdjust(double stepRPM, double targetWheelRPM){
        return Math.max(WHEEL_MIN_RPM, Math.min(WHEEL_MAX_RPM, targetWheelRPM + stepRPM)) / GEAR_RATIO;
    }

    public void launchStop(double power) {
        launchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchMotor.setPower(power);
    }

    public double getLaunchRPM() {
        return ticksPerSecToRpm(launchMotor.getVelocity()) * GEAR_RATIO;
    }
}
