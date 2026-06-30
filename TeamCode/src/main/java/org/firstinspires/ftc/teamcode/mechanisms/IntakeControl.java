package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeControl {
    private DcMotor intakeControl;

    public void init(HardwareMap hardwareMap) {
        intakeControl = hardwareMap.get(DcMotor.class, "Intake");
        intakeControl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeControl.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeControl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setIntakePower(double power) {
        intakeControl.setPower(power);
    }

    public double getIntakePower(){
        return intakeControl.getPower();
    }
}
