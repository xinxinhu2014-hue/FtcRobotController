package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private static final double TICK_PER_REV = 28 * 19.2;
    private static final double WHEEL_CIRCUM = 4 * 3.14;
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
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void drive(double forward, double right, double rotate) {
        double[] wheelsPower = new double[4];
        wheelsPower[0] = forward + right + rotate;
        wheelsPower[1] = forward - right - rotate;
        wheelsPower[2] = forward - right + rotate;
        wheelsPower[3] = forward + right - rotate;

        setPower(wheelsPower);
    }


    public void goStraight(double distance, double power) {
        double targetTick = distance / WHEEL_CIRCUM / GEAR_RATIO * TICK_PER_REV;

    }
}
