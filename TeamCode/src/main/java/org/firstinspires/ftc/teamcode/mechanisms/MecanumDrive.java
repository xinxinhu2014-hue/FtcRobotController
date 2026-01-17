package org.firstinspires.ftc.teamcode.mechanisms;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrive {
    private static final double ENCODER_TPR_MOTOR = 28.0; // 7 pulses per revolution per channel with quadrature decoding (×4)
    private static final double INTERNAL_GEAR = 19.2; // motor -> gearbox output
    private static final double TPR_OUTPUT = ENCODER_TPR_MOTOR * INTERNAL_GEAR; // TPR(Ticks per rev) for NeveRest 19.2:1 motor
    private static final double EXTERNAL_GEAR = 60.0 / 40.0; // Motor : Wheel = 60: 40
    private static final double MAX_MOTOR_RPM = 6600 / INTERNAL_GEAR; // RPM (Rev per min) for NeveRest 19.2:1 motor
    private static final double MAX_MOTOR_TICKS_PER_SEC = MAX_MOTOR_RPM * TPR_OUTPUT / 60;
    private static final double MIN_MOTOR_TICKS_PER_SEC = 0.1 * MAX_MOTOR_TICKS_PER_SEC;
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double IN_PER_REV = Math.PI * WHEEL_DIAMETER_IN * EXTERNAL_GEAR; // how many inches per motor rev
    private static final double TICKS_PER_INCH = TPR_OUTPUT / IN_PER_REV; // how many motor ticks per inch traveled
    private static final double kP = 10.0; // test out
    private static final double kI = 2.5; // test out
    private static final double kD = 0.0; // test out
    private static final double kF = 32767 / MAX_MOTOR_TICKS_PER_SEC; // REV firmware scaling. Best test out the actual MAX_MOTOR_TICKS_PER_SEC
    private static final double kP_HEADING = 0.015; // 0.010 ~ 0.030 typical; raise if it under-corrects
    private static final double kP_TURN = 0.012; // 0.010 ~ 0.020 typical; increase if turn is sluggish
    private static final double TOLERANCE_DEG = 1.5; // how close is “good enough” in turning

    private final DcMotorEx[] motors = new DcMotorEx[4];

    public void init(HardwareMap hardwareMap) {
        motors[0] = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        motors[1] = hardwareMap.get(DcMotorEx.class, "FrontRight");
        motors[2] = hardwareMap.get(DcMotorEx.class, "BackLeft");
        motors[3] = hardwareMap.get(DcMotorEx.class, "BackRight");

        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[3].setDirection(DcMotor.Direction.FORWARD);
        motors[1].setDirection(DcMotor.Direction.FORWARD);

        for (int i = 0; i < 4; i++) {
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
    }

    private double rpmToTicksPerSec(double rpm) {
        return rpm * TPR_OUTPUT / 60;
    }

    private double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / TPR_OUTPUT;
    }

    private double angleWrapDeg(double d) {
        while (d > 180) d -= 360;
        while (d <= -180) d += 360;
        return d;
    }

    // normalize the power input and then set
    private void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower,
                           double backRightPower) {
        double maxSpeed = 1.0;
        double[] powers = new double[]{frontLeftPower, frontRightPower, backLeftPower, backRightPower};
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, Math.abs(powers[i]));
        }

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i] / maxSpeed);
        }
    }

    public double[] getWheelRpm() {
        double[] wheelRpm = new double[4];
        for (int i = 0; i < 4; i++) {
            wheelRpm[i] = ticksPerSecToRpm(motors[i].getVelocity() * EXTERNAL_GEAR);
        }
        return wheelRpm;
    }

    public double percentMaxRpm(double percentMaxRpm) {
        return percentMaxRpm * MAX_MOTOR_RPM;
    }

    public void runUsingEncoders() {
        for (int i = 0; i < 4; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean isMotorBusy() {
        boolean isBusy = false;
        for (int i = 0; i < 4; i++) {
            isBusy = isBusy || motors[i].isBusy();
        }
        return isBusy;
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void stopDrive() {
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(0);
        }
    }

    public void driveForward(double wheelRpm) {
        for (int i = 0; i < 4; i++) {
            motors[i].setVelocity(rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        }
    }

    public void driveBackward(double wheelRpm) {
        for (int i = 0; i < 4; i++) {
            motors[i].setVelocity(-rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        }
    }

    public void rotateRight(double wheelRpm) {
        for (int i = 0; i < 2; i++) {
            motors[2 * i].setVelocity(rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
            motors[2 * i + 1].setVelocity(-rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        }
    }

    public void rotateLeft(double wheelRpm) {
        for (int i = 0; i < 2; i++) {
            motors[2 * i].setVelocity(-rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
            motors[2 * i + 1].setVelocity(rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        }
    }

    public void strafeRight(double wheelRpm) {
        motors[0].setVelocity(rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        motors[1].setVelocity(-rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        motors[2].setVelocity(-rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        motors[3].setVelocity(rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
    }

    public void strafeLeft(double wheelRpm) {
        motors[0].setVelocity(-rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        motors[1].setVelocity(rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        motors[2].setVelocity(rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
        motors[3].setVelocity(-rpmToTicksPerSec(wheelRpm / EXTERNAL_GEAR));
    }

    public double forwardRunToTargetPosition(double inches, double baseRPM) {
        int targetTicks = (int) Math.round(inches * TICKS_PER_INCH);
        for (int i = 0; i < 4; i++) {
            motors[i].setTargetPosition(motors[i].getCurrentPosition() + targetTicks);
            motors[i].setTargetPositionTolerance(10);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        return rpmToTicksPerSec(baseRPM);
    }

    public void forwardAdjustYawError(double yawError, double baseVelocityTps) {
        double err = angleWrapDeg(yawError);
        double turnBias = kP_HEADING * err * Math.abs(baseVelocityTps);
        double leftVel = baseVelocityTps - turnBias;
        double rightVel = baseVelocityTps + turnBias;
        leftVel = Range.clip(leftVel, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        rightVel = Range.clip(rightVel, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        for (int i = 0; i < 2; i++) {
            motors[2 * i].setVelocity(leftVel);
            motors[2 * i + 1].setVelocity(rightVel);
        }
    }

    public double strafeRunToTargetPosition(double inches, double baseRPM, boolean right) {
        int targetTicks = (int) Math.round(Math.abs(inches) * TICKS_PER_INCH);
        if (!right) targetTicks = - targetTicks; // left as negative in target pattern
        motors[0].setTargetPosition(motors[0].getCurrentPosition() + targetTicks);
        motors[1].setTargetPosition(motors[1].getCurrentPosition() - targetTicks);
        motors[2].setTargetPosition(motors[2].getCurrentPosition() - targetTicks);
        motors[3].setTargetPosition(motors[3].getCurrentPosition() + targetTicks);
        for (int i = 0; i < 4; i++) {
            motors[i].setTargetPositionTolerance(10);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        return rpmToTicksPerSec(baseRPM);
    }

    public void strafeAdjustYawError(double yawError, double baseVelocityTps) {
        double err = angleWrapDeg(yawError);
        double turnBias = kP_HEADING * err * Math.abs(baseVelocityTps);
        double[] adjustedVelocity = new double[4];
        adjustedVelocity[0] = Range.clip(baseVelocityTps - turnBias, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        adjustedVelocity[1] = Range.clip(- baseVelocityTps - turnBias, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        adjustedVelocity[2] = Range.clip(- baseVelocityTps + turnBias, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        adjustedVelocity[3] = Range.clip(baseVelocityTps + turnBias, -MAX_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);

        for (int i = 0; i < 4; i++) {
            motors[i].setVelocity(adjustedVelocity[i]);
        }
    }

    public boolean turnAdjustYawErr(double yawErr, double velPercent) {
        double err = angleWrapDeg(yawErr);

        // Proportional velocity command (scale by MAX_MOTOR_TICKS_PER_SEC)
        double turnVel = kP_TURN * Math.abs(err) * MAX_MOTOR_TICKS_PER_SEC * velPercent;

        // ensure we overcome stiction but don’t exceed limits
        if (Math.abs(err) > TOLERANCE_DEG) {
            turnVel = Range.clip(turnVel, MIN_MOTOR_TICKS_PER_SEC, MAX_MOTOR_TICKS_PER_SEC);
        } else {
            return true; // Robot is in the tolerance range
        }

        // Positive error => CCW: left backward, right forward
        double leftVel  = (err > 0) ? -turnVel :  turnVel;
        double rightVel = (err > 0) ?  turnVel : -turnVel;
        for (int i = 0; i < 2; i++) {
            motors[2 * i].setVelocity(leftVel);
            motors[2 * i + 1].setVelocity(rightVel);
        }
        return false; // Robot is still not in the tolerance range
    }

    public double setHeadingDeg(double currentYaw, double deltaYaw) {
        return angleWrapDeg(currentYaw + deltaYaw);
    }

}
