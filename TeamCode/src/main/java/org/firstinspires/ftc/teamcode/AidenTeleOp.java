package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ReleaseDoors;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeControl;

@TeleOp(name = "Competition TeleOp with RPM Control")
public class AidenTeleOp extends OpMode {
    // Your existing mechanism classes
    MecanumDrive drive = new MecanumDrive();
    ReleaseDoors bar = new ReleaseDoors();
    IntakeControl intake = new IntakeControl();

    // Shooter motor - directly controlled for RPM management
    private DcMotorEx shooterMotor;

    // Control states
    private boolean launching = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRightBumper = false;
    private double wheelTargetRpm = 0.0;
    private final double wheelRpmAdjustment = 100.0;
    private double launchRpm;
    private boolean barExtended = false;

    // RPM Control parameters
    private final double rpmTolerance = 70.0;
    private final double maxOvershootLimit = 100.0;
    private boolean shooterReady = false;

    // Shooter motor configuration - adjust for your specific motor
    private static final double SHOOTER_TICKS_PER_REV = 537.6; // REV HD Hex 20:1
    private static final double SHOOTER_MAX_RPM = 312.5; // REV HD Hex 20:1
    private static final double MAX_SAFE_RPM = SHOOTER_MAX_RPM * 0.95;

    // PIDF coefficients for velocity control
    private static final double kF = 32767.0 / (SHOOTER_MAX_RPM * SHOOTER_TICKS_PER_REV / 60.0);
    private static final double kP = 0.10 * kF;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Performance monitoring
    private long lastLoopTime = 0;
    private double loopFrequency = 0;
    private int loopCount = 0;

    @Override
    public void init() {
        // Initialize your existing mechanisms
        drive.init(hardwareMap);
        bar.init(hardwareMap);
        intake.init(hardwareMap);

        // Initialize shooter motor directly for precise RPM control
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter"); // Use your actual motor name

        // Configure shooter motor for velocity control
        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Apply PIDF coefficients for precise velocity control
        PIDFCoefficients coefficients = new PIDFCoefficients(kP, kI, kD, kF);
        shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);

        // Bulk caching optimization for better performance
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addData("Status", "Initialized - RPM Control Active");
        telemetry.addData("Max Safe RPM", "%.0f", MAX_SAFE_RPM);
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset push bar to starting position
        bar.openDoor(0.65, 1.0);
    }

    private double dead(double v){
        return Math.abs(v) < 0.05 ? 0.0 : v;
    }

    // Convert RPM to encoder ticks per second for velocity control
    private double rpmToTicksPerSec(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV) / 60.0;
    }

    // Convert ticks per second to RPM for reading current speed
    private double ticksPerSecToRPM(double tps) {
        return (tps * 60.0) / SHOOTER_TICKS_PER_REV;
    }

    // Check if shooter is within target RPM range
    private boolean isShooterReady(double currentRpm, double targetRpm) {
        return Math.abs(currentRpm - targetRpm) <= rpmTolerance;
    }

    // Smart RPM management to prevent overshoot and maintain target
    private void manageShooterRPM() {
        if (launching && shooterMotor != null) {
            // Get current RPM
            launchRpm = ticksPerSecToRPM(shooterMotor.getVelocity());

            // Check if we're within target range
            shooterReady = isShooterReady(launchRpm, wheelTargetRpm);

            // Safety: Prevent excessive overshoot
            if (launchRpm > wheelTargetRpm + maxOvershootLimit) {
                // Reduce velocity temporarily to prevent overshoot
                double correctedVelocity = rpmToTicksPerSec(wheelTargetRpm - 50);
                shooterMotor.setVelocity(correctedVelocity);
            }

            // Safety: Limit maximum RPM to prevent motor damage
            if (launchRpm > MAX_SAFE_RPM) {
                shooterMotor.setVelocity(rpmToTicksPerSec(MAX_SAFE_RPM));
            }
        }
    }

    // Set shooter to target RPM with safety checks
    private void setShooterRPM(double targetRPM) {
        if (targetRPM > MAX_SAFE_RPM) {
            targetRPM = MAX_SAFE_RPM;
            telemetry.addData("WARNING", "RPM limited to safe maximum: %.0f", MAX_SAFE_RPM);
        }

        wheelTargetRpm = targetRPM;
        double targetTicksPerSec = rpmToTicksPerSec(wheelTargetRpm);
        shooterMotor.setVelocity(targetTicksPerSec);
        launching = true;
        shooterReady = false; // Reset ready state when changing target
    }

    @Override
    public void loop() {
        loopCount++;

        // Calculate loop frequency for performance monitoring
        long currentTime = System.nanoTime();
        if (lastLoopTime != 0) {
            loopFrequency = 1e9 / (currentTime - lastLoopTime);
        }
        lastLoopTime = currentTime;

        // ===================== DRIVE CONTROL =====================
        double forward = gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        forward = dead(forward);
        right = dead(right);
        rotate = dead(rotate);
        drive.drive(forward, right, rotate);

        // ===================== PUSH BAR CONTROL =====================
        boolean rightBumper = gamepad1.right_bumper;
        if (rightBumper && !lastRightBumper) {
            barExtended = !barExtended;
            if (barExtended) {
                bar.openDoor(0.55, 0.2);
            } else {
                bar.closeDoor(0.65, 1.0);
            }
        }
        lastRightBumper = rightBumper;

        // ===================== INTAKE CONTROL =====================
        if (gamepad1.left_bumper) {
            intake.setIntakePower(-1.0); // Reverse to clear jams
        } else {
            intake.setIntakePower(gamepad1.left_trigger); // Forward intake
        }

        // ===================== SMART SHOOTER CONTROL =====================
        // Manage RPM to prevent overshoot and check ready status
        manageShooterRPM();

        // RPM adjustment while active
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp && launching) {
            double newTarget = wheelTargetRpm + wheelRpmAdjustment;
            setShooterRPM(Math.min(5000, newTarget));
        }
        lastDpadUp = dpadUp;

        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !lastDpadDown && launching) {
            double newTarget = wheelTargetRpm - wheelRpmAdjustment;
            setShooterRPM(Math.max(1000, newTarget)); // Minimum 1000 RPM
        }
        lastDpadDown = dpadDown;

        // Preset RPM buttons
        if (gamepad1.a && !launching) {
            setShooterRPM(3800.0); // High power - long distance
        }

        if (gamepad1.b && !launching) {
            setShooterRPM(3400.0); // Medium power - medium distance
        }

        if (gamepad1.x && !launching) {
            setShooterRPM(2900.0); // Low power - short distance
        }

        // Stop shooter
        if (gamepad1.y && launching) {
            shooterMotor.setVelocity(0);
            launching = false;
            shooterReady = false;
        }

        // Update current RPM reading
        if (shooterMotor != null) {
            launchRpm = ticksPerSecToRPM(shooterMotor.getVelocity());
        }

        // ===================== ADVANCED COMPETITION TELEMETRY =====================
        telemetry.addLine("=== DRIVE SYSTEM ===");
        telemetry.addData("Input", "F:%.2f R:%.2f T:%.2f", forward, right, rotate);

        telemetry.addLine("=== MECHANISMS ===");
        telemetry.addData("Intake", "%.2f", intake.getIntakePower());
        telemetry.addData("Push Bar", barExtended ? "EXTENDED" : "RETRACTED");

        telemetry.addLine("=== SMART SHOOTER SYSTEM ===");
        telemetry.addData("Status", launching ? (shooterReady ? "READY TO FIRE!" : "SPINNING UP...") : "STANDBY");
        telemetry.addData("Target RPM", "%.0f", wheelTargetRpm);
        telemetry.addData("Actual RPM", "%.0f", launchRpm);
        telemetry.addData("RPM Difference", "%.0f", Math.abs(launchRpm - wheelTargetRpm));
        telemetry.addData("Accuracy", "%.1f%%", wheelTargetRpm > 0 ? (launchRpm / wheelTargetRpm) * 100 : 0);
        telemetry.addData("Within Tolerance", shooterReady ? "YES (±" + rpmTolerance + " RPM)" : "NO");
        telemetry.addData("Control Mode", "PIDF Velocity Control");

        // Visual competition-ready indicator
        if (launching) {
            if (shooterReady) {
                telemetry.addLine("🎯 *** READY TO SHOOT! *** 🎯");
            } else {
                double progress = Math.min(1.0, launchRpm / wheelTargetRpm);
                telemetry.addData("Spin-up Progress", "%.0f%%", progress * 100);
            }
        }

        telemetry.addLine("=== PERFORMANCE ===");
        telemetry.addData("Loop Freq", "%.1f Hz", loopFrequency);
        telemetry.addData("Loop Count", "%d", loopCount);
        telemetry.addData("Battery", "%.1fV", hardwareMap.voltageSensor.iterator().next().getVoltage());

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A/B/X", "Preset: 3800/3400/2900 RPM");
        telemetry.addData("Y", "Stop Shooter");
        telemetry.addData("D-Pad", "Adjust RPM ±100");
        telemetry.addData("RB", "Toggle Push Bar");
        telemetry.addData("LB/Trigger", "Intake Out/In");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Safety: Stop shooter when op mode ends
        if (shooterMotor != null) {
            shooterMotor.setVelocity(0);
        }
        super.stop();
    }
}