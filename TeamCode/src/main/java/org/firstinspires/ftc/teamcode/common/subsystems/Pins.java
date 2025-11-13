package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pins {
    private final Servo leftPin;
    private final Servo centerPin;
    private final Servo rightPin;

    private final static double leftPinClose = 0.325;
    private final static double centerPinClose = 0.435;
    private final static double rightPinClose = 0.8;

    private final static double leftPinOpen= 0.375;
    private final static double centerPinOpen = 0.65;
    private final static double rightPinOpen = 0.725;

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
