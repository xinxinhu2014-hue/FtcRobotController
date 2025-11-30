package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherControl {


    private static final double TICKS_PER_REV = 28.0; // = 7 * 4 for NeveRest 1:1 motor
    private static final double GEAR_RATIO = 1.0;
    private static final double MAX_MOTOR_RPM = 6600.0; // for NeveRest 1:1 motor
    private static final double MAX_WHEEL_RPM = MAX_MOTOR_RPM * GEAR_RATIO;
    private static final double MIN_WHEEL_RPM = 2000.0; // test out when shooting in shortest range
    private static final double kP = 0.08; // test out
    private static final double kI = 0.0; // test out
    private static final double kD = 0.0; // test out
    private static final double kF = 1.2 * 32767.0 / (MAX_MOTOR_RPM * TICKS_PER_REV / 60.0);
    private DcMotorEx launchMotor;



    public void init(HardwareMap hardwareMap){
        launchMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        launchMotor.setDirection(DcMotor.Direction.REVERSE);
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //launchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    double rpmToTicksPerSec(double rpm) { return rpm * TICKS_PER_REV / 60; }
    double ticksPerSecToRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }

    // Read wheel RPM from current motor velocity
    public double currentWheelRpm() {
        double motorTps = launchMotor.getVelocity();                   // motor tps
        return ticksPerSecToRpm(motorTps) * GEAR_RATIO;                // wheel rpm
    }

    public void stopLaunch() {
        launchMotor.setVelocity(0);
    }

    public void startLaunch(double desiredWheelRpm) {
        double wheelRpm = Math.min(MAX_WHEEL_RPM, Math.max(MIN_WHEEL_RPM, desiredWheelRpm));
        double motorRpm = wheelRpm / GEAR_RATIO;
        double kFscale, kPadjust = 0.0;
        if (wheelRpm > 3600) {
            kFscale = 1.0515;          // extra push only for very high RPM
            kPadjust = 0.1;
        } else if (wheelRpm > 3200) {
            kFscale = 1.05; // 1.04
        } else {
            kFscale = 1.05; // 1.0
        }
        launchMotor.setVelocityPIDFCoefficients(kP + kPadjust, kI, kD, kF * kFscale);
        launchMotor.setVelocity(rpmToTicksPerSec(motorRpm));
    }

    public double adjustLaunchRpm(double currentWheelRpm, double rpmChange) {
        return Math.min(MAX_WHEEL_RPM, Math.max(MIN_WHEEL_RPM, currentWheelRpm + rpmChange));
    }

}
