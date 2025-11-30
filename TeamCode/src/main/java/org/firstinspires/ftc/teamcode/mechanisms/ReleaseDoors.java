package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.List;
import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ReleaseDoors {
    private Servo leftDoor;
    private Servo rightDoor;

    public void init(HardwareMap hardwareMap) {
        leftDoor = hardwareMap.get(Servo.class, "LeftDoor");
        rightDoor = hardwareMap.get(Servo.class, "RightDoor");
    }

    public List<Double> getBarPosition() {
        List<Double> barPosition = new ArrayList<Double>();

        barPosition.add(leftDoor.getPosition());
        barPosition.add(rightDoor.getPosition());

        return barPosition;
    }
    public void openDoor(double leftBarPosition, double rightBarPosition) {
        leftDoor.setDirection(Servo.Direction.FORWARD);
        rightDoor.setDirection(Servo.Direction.REVERSE);
        leftDoor.setPosition(leftBarPosition);
        rightDoor.setPosition(rightBarPosition);
    }

    public void closeDoor(double leftBarPosition, double rightBarPosition) {
        leftDoor.setDirection(Servo.Direction.REVERSE);
        rightDoor.setDirection(Servo.Direction.FORWARD);
        leftDoor.setPosition(leftBarPosition);
        rightDoor.setPosition(rightBarPosition);
    }



}


