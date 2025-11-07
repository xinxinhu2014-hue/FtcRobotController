package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeControl {
    private DcMotor intakeControl;

    public void init(HardwareMap hardwareMap) {
        intakeControl = hardwareMap.get(DcMotor.class, "intake");
        intakeControl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeControl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIntakePower(double power) {
        intakeControl.setPower(power);
    }
}
