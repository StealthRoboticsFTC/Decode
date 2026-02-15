package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lifter {
    private final Servo leftLift;
    private final Servo rightLift;
    private final static double liftUpPos = 0.075;

    private final static double liftBlockPos = 0.15;
    private final static double leftDownPos= 0.275;


    public Lifter(HardwareMap hardwareMap){
        leftLift = hardwareMap.get(Servo.class, "servo_ll");
        rightLift =  hardwareMap.get(Servo.class, "servo_rl");
        rightLift.setDirection(Servo.Direction.REVERSE);
    }

    public void liftUp(){
        leftLift.setPosition(liftUpPos);
        rightLift.setPosition(liftUpPos);
    }

    public void liftDown(){
        leftLift.setPosition(leftDownPos);
        rightLift.setPosition(leftDownPos);
    }

    public void liftBlock(){
        leftLift.setPosition(liftBlockPos);
        rightLift.setPosition(liftBlockPos);
    }
}
