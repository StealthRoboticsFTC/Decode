package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pins {
    private final Servo leftPin;
    private final Servo centerPin;
    private final Servo rightPin;

    private final static double leftPinClose = 0.25;
    private final static double centerPinClose = 0.5;
    private final static double rightPinClose = 0.75;

    private final static double leftPinOpen= 0.75;
    private final static double centerPinOpen = 0.5;
    private final static double rightPinOpen = 0.25;

    public Pins(HardwareMap hardwareMap){
        leftPin = hardwareMap.get(Servo.class,"servo_lp");
        centerPin = hardwareMap.get(Servo.class,"servo_cp");
        rightPin = hardwareMap.get(Servo.class,"servo_rp");
    }

    public void closePin(){
        leftPin.setPosition(leftPinClose);
        rightPin.setPosition(rightPinClose);
        centerPin.setPosition(centerPinClose);
    }

    public void openPin(){
        leftPin.setPosition(leftPinOpen);
        rightPin.setPosition(rightPinOpen);
        centerPin.setPosition(centerPinOpen);
    }


}
