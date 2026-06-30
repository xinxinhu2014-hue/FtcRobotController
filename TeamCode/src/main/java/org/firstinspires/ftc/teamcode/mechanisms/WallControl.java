package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.List;
import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WallControl {
    private Servo leftWall;
    private Servo rightWall;

    public void init(HardwareMap hardwareMap) {
        leftWall = hardwareMap.get(Servo.class, "LeftWall");
        rightWall = hardwareMap.get(Servo.class, "RightWall");
    }

    public List<Double> getWallPosition() {
        List<Double> wallPosition = new ArrayList<Double>();

        wallPosition.add(leftWall.getPosition());
        wallPosition.add(rightWall.getPosition());

        return wallPosition;
    }
    public void loosenWall(double leftWallPosition, double rightWallPosition) {
        leftWall.setDirection(Servo.Direction.FORWARD);
        rightWall.setDirection(Servo.Direction.REVERSE);
        leftWall.setPosition(leftWallPosition);
        rightWall.setPosition(rightWallPosition);
    }

    public void tightenWall(double leftWallPosition, double rightWallPosition) {
        leftWall.setDirection(Servo.Direction.REVERSE);
        rightWall.setDirection(Servo.Direction.FORWARD);
        leftWall.setPosition(leftWallPosition);
        rightWall.setPosition(rightWallPosition);
    }



}


