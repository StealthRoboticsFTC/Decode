package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pins {
    private final Servo leftPin;
    private final Servo centerPin;
    private final Servo rightPin;

    private final static double leftPinClose = 0;
    private final static double centerPinClose = 1;
    private final static double rightPinClose = 0.55;

    private final static double leftPinOpen= 0.15;
    private final static double centerPinOpen = 0.8;
    private final static double rightPinOpen = 0.45;

    Servo[] pins;

    Double[] open = {leftPinOpen, centerPinOpen, rightPinOpen};

    public Pins(HardwareMap hardwareMap){
        leftPin = hardwareMap.get(Servo.class,"servo_lp");
        centerPin = hardwareMap.get(Servo.class,"servo_cp");
        rightPin = hardwareMap.get(Servo.class,"servo_rp");
        pins = new Servo[]{leftPin, centerPin, rightPin};
    }

    public void closeAllPin(){
        leftPin.setPosition(leftPinClose);
        rightPin.setPosition(rightPinClose);
        centerPin.setPosition(centerPinClose);
    }

    public void openAllPin(){
        leftPin.setPosition(leftPinOpen);
        rightPin.setPosition(rightPinOpen);
        centerPin.setPosition(centerPinOpen);
    }

    public void setPinsToIntake(){
        leftPin.setPosition(leftPinClose);
        rightPin.setPosition(rightPinClose);
    }

    public void setPinOpen(int index){
        pins[index].setPosition(open[index]);
    }


}
