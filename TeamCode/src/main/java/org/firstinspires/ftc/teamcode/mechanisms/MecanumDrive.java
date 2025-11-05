package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private static final double TICK_PER_REV = 28 * 19.2;
    private static final double WHEEL_CIRCUM = 4 * Math.PI;
    private static final double GEAR_RATIO = 60.0/40.0;
    private final DcMotor[] wheelsMotor = new DcMotor[4];


    public void init(HardwareMap hardwareMap) {
        wheelsMotor[0] = hardwareMap.dcMotor.get("frontleft");
        wheelsMotor[1] = hardwareMap.dcMotor.get("frontright");
        wheelsMotor[2] = hardwareMap.dcMotor.get("backleft");
        wheelsMotor[3] = hardwareMap.dcMotor.get("backright");

        wheelsMotor[0].setDirection(DcMotor.Direction.REVERSE);
        wheelsMotor[2].setDirection(DcMotor.Direction.REVERSE);
        wheelsMotor[1].setDirection(DcMotor.Direction.FORWARD);
        wheelsMotor[3].setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : wheelsMotor) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }

    private void setPower(double[] wheelsPower) {
        double maxSpeed = 1.0;
        for (double power : wheelsPower) {
            maxSpeed = Math.max(maxSpeed, Math.abs(power));
        }
        for (int i = 0; i < 4; i++) {
            wheelsPower[i] /= maxSpeed;
            wheelsMotor[i].setPower(wheelsPower[i]);
        }
    }

    public void drive(double forward, double side, double rotate) {
        double[] wheelsPower = new double[4];
        wheelsPower[0] = forward + side + rotate;
        wheelsPower[1] = forward - side - rotate;
        wheelsPower[2] = forward - side + rotate;
        wheelsPower[3] = forward + side - rotate;

        setPower(wheelsPower);
    }

    public void goStraight(double distance, double power) {
        int targetTick = (int) (distance / WHEEL_CIRCUM / GEAR_RATIO * TICK_PER_REV);
        int dirSign;
        for (int i = 0; i < 4; i++) {
            dirSign = (wheelsMotor[i].getDirection() == DcMotor.Direction.REVERSE) ? -1 : 1;
            wheelsMotor[i].setTargetPosition(targetTick * dirSign);
            wheelsMotor[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (int i = 0; i < 4; i++) {
            wheelsMotor[i].setPower(power);
        }

        while (wheelsMotor[0].isBusy() || wheelsMotor[1].isBusy() || wheelsMotor[2].isBusy() || wheelsMotor[3].isBusy()) {
            // optional: add telemetry here
        }

        // Stop and hold
        for (DcMotor motor : wheelsMotor) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }
}
