package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherControl {


    private static final double TICKS_PER_REV = 28.0; // = 7 * 4 for NeveRest 1:1 motor
    private static final double GEAR_RATIO = 24.0/14.0; // motor to wheel
    private static final double MAX_MOTOR_RPM = 6600.0; // for NeveRest 1:1 motor
    private static final double MAX_WHEEL_RPM = MAX_MOTOR_RPM * GEAR_RATIO;
    private static final double MIN_WHEEL_RPM = 2000.0; // test out when shooting in shortest range
    private static final double kP = 0.08; // test out
    private static final double kI = 0.0; // test out
    private static final double kD = 0.0; // test out
    private static final double kF = 32767.0 / (MAX_MOTOR_RPM * TICKS_PER_REV / 60.0);
    private DcMotorEx[] launchMotors = new DcMotorEx[2];



    public void init(HardwareMap hardwareMap){
        launchMotors[0] = hardwareMap.get(DcMotorEx.class, "FrontLaunch");
        launchMotors[1] = hardwareMap.get(DcMotorEx.class, "BackLaunch");
        for (int i = 0; i <2; i++)  {
            launchMotors[i].setDirection(DcMotor.Direction.REVERSE);
            launchMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launchMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launchMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    double rpmToTicksPerSec(double rpm) { return rpm * TICKS_PER_REV / 60; }
    double ticksPerSecToRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }

    // Read wheel RPM from current motor velocity
    public double currentWheelRpm() {
        double motorTps = (launchMotors[0].getVelocity() + launchMotors[1].getVelocity()) / 2;  // motor tps
        return ticksPerSecToRpm(motorTps) * GEAR_RATIO;                // wheel rpm
    }

    public void stopLaunch() {
        for(int i = 0; i < 2; i++) {
            launchMotors[i].setVelocity(0);
        }
    }

    public void startLaunch(double desiredWheelRpm) {
        double wheelRpm = Math.min(MAX_WHEEL_RPM, Math.max(MIN_WHEEL_RPM, desiredWheelRpm));
        double motorRpm = wheelRpm / GEAR_RATIO;
        double kFscale, kPadjust = 0.0;
        if (wheelRpm > 3600) {
            kFscale = 1.62;          // extra push only for very high RPM
            kPadjust = 0.1;
        } else if (wheelRpm > 3200) {
            kFscale = 1.58;
            kPadjust = 0.08;
        } else {
            kFscale = 1.58;
            kPadjust = 0.04;
        }
        for (int i = 0; i < 2; i++) {
            launchMotors[i].setVelocityPIDFCoefficients(kP + kPadjust, kI, kD, kF * kFscale);
            launchMotors[i].setVelocity(rpmToTicksPerSec(motorRpm));
        }
    }

    public double adjustLaunchRpm(double currentWheelRpm, double rpmChange) {
        return Math.min(MAX_WHEEL_RPM, Math.max(MIN_WHEEL_RPM, currentWheelRpm + rpmChange));
    }

}
