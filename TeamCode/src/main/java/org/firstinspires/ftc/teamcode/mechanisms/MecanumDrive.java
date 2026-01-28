package org.firstinspires.ftc.teamcode.mechanisms;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrive {
    private static final double ENCODER_TPR_MOTOR = 28.0; // 7 pulses per revolution per channel with quadrature decoding (×4)
    private static final double INTERNAL_GEAR = 19.2; // motor -> gearbox output
    private static final double TPR_OUTPUT = ENCODER_TPR_MOTOR * INTERNAL_GEAR; // TPR(Ticks per rev) for NeveRest 19.2:1 motor = 537.6
    private static final double EXTERNAL_GEAR = 60.0 / 40.0; // Motor : Wheel = 60: 40 = 1.5

    private static final double MAX_MOTOR_RPM = 6600 / INTERNAL_GEAR; // RPM (Rev per min) for NeveRest 19.2:1 motor = 343.75
    private static final double MAX_MOTOR_TICKS_PER_SEC = MAX_MOTOR_RPM * TPR_OUTPUT / 60; // = 3080
    private static final double MIN_MOTOR_TICKS_PER_SEC = 0.1 * MAX_MOTOR_TICKS_PER_SEC; // = 308
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double IN_PER_REV = Math.PI * WHEEL_DIAMETER_IN * EXTERNAL_GEAR; // how many inches per motor rev = 18.85
    private static final double TICKS_PER_INCH = TPR_OUTPUT / IN_PER_REV; // how many motor ticks per inch traveled = 28.52
    private static final double TURN_TRIM_TPS = 0; // + = steer right, - = steer left
    private static final double kP = 10.0; // test out
    private static final double kI = 2.5; // test out
    private static final double kD = 0.0; // test out
    private static final double kF = 32767 / MAX_MOTOR_TICKS_PER_SEC; // REV firmware scaling. Best test out the actual MAX_MOTOR_TICKS_PER_SEC
    private static final double kP_HEADING = 30; // ticks/sec per degree
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

        // deadband helps at low speed
        if (Math.abs(err) < 1.0) err = 0.0;

        // heading correction
        double turnBias = kP_HEADING * err;

        // OPTIONAL: only add trim in a controlled way (or comment out while tuning)
        // turnBias += Math.signum(err) * TURN_TRIM_TPS;

        turnBias = Range.clip(
                turnBias,
                -0.2 * MAX_MOTOR_TICKS_PER_SEC,
                0.2 * MAX_MOTOR_TICKS_PER_SEC
        );

        double leftVel  = baseVelocityTps - turnBias;
        double rightVel = baseVelocityTps + turnBias;

        double max = Math.max(Math.abs(leftVel), Math.abs(rightVel));
        if (max > MAX_MOTOR_TICKS_PER_SEC) {
            double s = MAX_MOTOR_TICKS_PER_SEC / max;
            leftVel *= s;
            rightVel *= s;
        }

        motors[0].setVelocity(leftVel);
        motors[1].setVelocity(rightVel);
        motors[2].setVelocity(leftVel);
        motors[3].setVelocity(rightVel);
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

        double rot = kP_HEADING * err;
        rot = Range.clip(rot,
                -0.2 * MAX_MOTOR_TICKS_PER_SEC,
                0.2 * MAX_MOTOR_TICKS_PER_SEC);

        // base strafe (right if baseVelocityTps > 0)
        double fl =  baseVelocityTps;
        double fr = -baseVelocityTps;
        double bl = -baseVelocityTps;
        double br =  baseVelocityTps;

        // add rotation (NOT front vs back)
        fl += rot;
        fr -= rot;
        bl += rot;
        br -= rot;

        // normalize if needed
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > MAX_MOTOR_TICKS_PER_SEC) {
            double s = MAX_MOTOR_TICKS_PER_SEC / max;
            fl *= s; fr *= s; bl *= s; br *= s;
        }

        motors[0].setVelocity(fl);
        motors[1].setVelocity(fr);
        motors[2].setVelocity(bl);
        motors[3].setVelocity(br);
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

    // Below from Chat GPT

    // --- Option A state ---
    private final int[] startPos = new int[4];
    private int driveTargetTicks = 0;
    private boolean driveActive = false;

    private void rememberStartPositions() {
        for (int i = 0; i < 4; i++) startPos[i] = motors[i].getCurrentPosition();
    }

    private int avgAbsDeltaFromStart() {
        int sum = 0;
        for (int i = 0; i < 4; i++) sum += Math.abs(motors[i].getCurrentPosition() - startPos[i]);
        return sum / 4;
    }

    private double computeBrakeZoneInches(
            double inches,
            double fraction,
            double minBrakeIn,
            double maxBrakeIn
    ) {
        double dist = Math.abs(inches);
        double brakeZone = fraction * dist;

        if (brakeZone < minBrakeIn) brakeZone = minBrakeIn;
        if (brakeZone > maxBrakeIn) brakeZone = maxBrakeIn;

        return brakeZone;
    }


    public void beginForwardDistanceInches(double inches) {
        runUsingEncoders();              // switch to RUN_USING_ENCODER
        rememberStartPositions();
        driveTargetTicks = (int) Math.round(Math.abs(inches) * TICKS_PER_INCH);
        driveActive = true;
    }
    public double getAverageEncoderTicks() {
        double sumTicks = 0;
        for(int i = 0; i < 4; i++) sumTicks += Math.abs(motors[i].getCurrentPosition() - startPos[i]);

        return sumTicks / 4.0;
    }

    public double getActualInches() {
        return getAverageEncoderTicks() / TICKS_PER_INCH;
    }

    public boolean updateForwardDistanceHoldHeading(
            double inches,
            double targetHeadingDeg,
            double currentHeadingDeg,
            double baseMotorPct,
            double minMotorPct
    ) {
        if (!driveActive) beginForwardDistanceInches(inches);

        int progressed = avgAbsDeltaFromStart();
        int remaining = driveTargetTicks - progressed;

        if (remaining <= 0) {
            driveActive = false;
            return true;
        }

        int slowZoneTicks = (int) Math.round(8.0 * TICKS_PER_INCH);

        double pct = baseMotorPct;
        if (remaining < slowZoneTicks) {
            double t = (double) remaining / slowZoneTicks;
            pct = minMotorPct + (baseMotorPct - minMotorPct) * t;
        }

        double motorRpm = pct * MAX_MOTOR_RPM;
        double baseTps = rpmToTicksPerSec(motorRpm);
        if (inches < 0) baseTps = -baseTps;

        double yawError = angleWrapDeg(targetHeadingDeg - currentHeadingDeg);
        forwardAdjustYawError(yawError, baseTps);

        return false;
    }

    public boolean updateForwardDistanceHoldHeadingSlowdown(
            double inches,
            double targetHeadingDeg,
            double currentHeadingDeg,
            double baseMotorPct,
            double minMotorPct,
            double fracBrake,
            double minBrakeIn,
            double maxBrakeIn
    ) {
        if (!driveActive) beginForwardDistanceInches(inches);

        int progressed = avgAbsDeltaFromStart();
        int remaining = driveTargetTicks - progressed;

        // --- DONE tolerance ---
        final int posTolTicks = (int) Math.round(0.6 * TICKS_PER_INCH); // tune 0.4–1.0
        if (remaining <= posTolTicks) {
            driveActive = false;
            return true;
        }

        // --- sanitize params ---
        if (baseMotorPct < 0) baseMotorPct = -baseMotorPct;
        if (minMotorPct < 0)  minMotorPct  = -minMotorPct;

        // min should never exceed base
        if (minMotorPct > baseMotorPct) minMotorPct = baseMotorPct;

        // fracBrake reasonable range
        if (fracBrake < 0.05) fracBrake = 0.05;
        if (fracBrake > 0.90) fracBrake = 0.90;

        // --- compute brake zone inches -> ticks ---
        double distAbsIn = Math.abs(inches);
        double brakeZoneIn = fracBrake * distAbsIn;
        if (brakeZoneIn < minBrakeIn) brakeZoneIn = minBrakeIn;
        if (brakeZoneIn > maxBrakeIn) brakeZoneIn = maxBrakeIn;

        int slowZoneTicks = (int) Math.round(brakeZoneIn * TICKS_PER_INCH);
        if (slowZoneTicks < 1) slowZoneTicks = 1;

        // --- Speed ramp ---
        double pct = baseMotorPct;

        int remForRamp = Math.max(remaining, 0);
        if (remForRamp < slowZoneTicks) {
            double t = (double) remForRamp / (double) slowZoneTicks; // 1 -> 0 near target
            // clamp t
            if (t < 0) t = 0;
            if (t > 1) t = 1;

            pct = minMotorPct + (baseMotorPct - minMotorPct) * t;

            // extra caps near target (optional, but good for 30lb mecanum)
            int sixInTicks   = (int) Math.round(6.0 * TICKS_PER_INCH);
            int threeInTicks = (int) Math.round(3.0 * TICKS_PER_INCH);

            // caps should not exceed baseMotorPct
            double cap6  = Math.min(0.30, baseMotorPct);
            double cap3  = Math.min(0.20, baseMotorPct);

            if (remForRamp < sixInTicks)   pct = Math.min(pct, cap6);
            if (remForRamp < threeInTicks) pct = Math.min(pct, cap3);
        }

        if (pct < minMotorPct) pct = minMotorPct;

        // --- Convert pct -> ticks/sec ---
        double motorRpm = pct * MAX_MOTOR_RPM;
        double baseTps = rpmToTicksPerSec(motorRpm);
        if (inches < 0) baseTps = -baseTps;

        // --- Heading hold ---
        double yawError = angleWrapDeg(targetHeadingDeg - currentHeadingDeg);
        forwardAdjustYawError(yawError, baseTps);

        return false;
    }



    public void cancelDistanceMove() {
        driveActive = false;
        driveTargetTicks = 0;
    }

    public void beginStrafeDistanceInches(double inches) {
        runUsingEncoders();
        rememberStartPositions();
        driveTargetTicks = (int) Math.round(Math.abs(inches) * TICKS_PER_INCH);
        driveActive = true;
    }

    public boolean updateStrafeDistanceHoldHeading(
            double inches,
            boolean right,
            double targetHeadingDeg,
            double currentHeadingDeg,
            double baseMotorPct,
            double minMotorPct
    ) {
        if (!driveActive) beginStrafeDistanceInches(inches);

        int progressed = avgAbsDeltaFromStart();
        int remaining = driveTargetTicks - progressed;

        if (remaining <= 0) {
            driveActive = false;
            return true;
        }

        int slowZoneTicks = (int) Math.round(8.0 * TICKS_PER_INCH);

        double pct = baseMotorPct;
        if (remaining < slowZoneTicks) {
            double t = (double) remaining / slowZoneTicks;
            pct = minMotorPct + (baseMotorPct - minMotorPct) * t;
        }

        double motorRpm = pct * MAX_MOTOR_RPM;
        double baseTps = rpmToTicksPerSec(motorRpm);
        if (!right) baseTps = -baseTps;

        double yawError = angleWrapDeg(targetHeadingDeg - currentHeadingDeg);
        strafeAdjustYawError(yawError, baseTps);

        return false;
    }


    public void setRawLeftRightTps(double leftTps, double rightTps) {
        motors[0].setVelocity(leftTps);   // FrontLeft
        motors[2].setVelocity(leftTps);   // BackLeft
        motors[1].setVelocity(rightTps);  // FrontRight
        motors[3].setVelocity(rightTps);  // BackRight
    }

    public void stopDriveVelocity() {
        for (int i = 0; i < 4; i++) {
            motors[i].setVelocity(0);
        }
    }

    public void setBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior z = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        for (int i = 0; i < 4; i++) motors[i].setZeroPowerBehavior(z);
    }

}
