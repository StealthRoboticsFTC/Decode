package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SortingTest extends LinearOpMode {

    public static int id = 21;
    private static int stage = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftPin = hardwareMap.get(Servo.class, "servo_lp");
        Servo centerPin = hardwareMap.get(Servo.class, "servo_cp");
        Servo rightPin = hardwareMap.get(Servo.class, "servo_rp");
        ColorSensor leftColorSensor = hardwareMap.get(ColorSensor.class, "sensor_lc");
        ColorSensor rightColorSensor = hardwareMap.get(ColorSensor.class, "sensor_rc");
        ColorSensor centerColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cc");

        boolean leftPurple = false;
        boolean leftGreen = false;
        boolean leftNo = true;
        boolean rightPurple = false;
        boolean rightGreen = false;
        boolean rightNo = true;
        boolean centerPurple = false;
        boolean centerGreen = false;
        boolean centerNo = true;

        waitForStart();

        while (!isStopRequested()){
            if (leftNo){
                leftGreen = false;
                leftPurple = false;
                if (leftColorSensor.blue()<100 && leftColorSensor.red()<100 && leftColorSensor.green()<100)   {
                    leftNo = true;

                } else if (leftColorSensor.green() > leftColorSensor.blue() && leftColorSensor.green() > leftColorSensor.red()) {
                    leftNo = false;
                    leftGreen = true;
                } else {
                    leftNo = false;
                    leftPurple = true;
                }
            }
            if (rightNo){
                rightGreen = false;
                rightPurple = false;
                if (rightColorSensor.blue()<100 && rightColorSensor.red()<100 && rightColorSensor.green()<100)   {
                    rightNo = true;
                } else if (rightColorSensor.green() > rightColorSensor.blue() && rightColorSensor.green() > rightColorSensor.red()) {
                    rightNo = false;
                    rightGreen= true;
                } else {
                    rightNo = false;
                    rightPurple = true;
                }
            }
            if (centerNo){
                centerGreen = false;
                centerPurple = false;
                if (centerColorSensor.blue()<100 &&centerColorSensor.red()<100 && centerColorSensor.green()<100)   {
                    centerNo = true;
                } else if (centerColorSensor.green() > centerColorSensor.blue() && centerColorSensor.green() > centerColorSensor.red()) {
                    centerNo = false;
                    centerGreen = true;
                } else {
                    centerNo = false;
                    centerPurple = true;
                }
            }
            if (id == 21){
                if (stage == 0){
                    if (leftGreen){
                        leftPin.setPosition(0);
                        leftNo = true;
                        stage++;
                    } else if (centerGreen) {
                        centerPin.setPosition(0);
                        centerNo = true;
                        stage++;
                    } else if (rightGreen) {
                        rightPin.setPosition(0);
                        rightNo = true;
                        stage++;
                    }

                } else if (stage == 1) {
                    if (leftPurple){
                        leftPin.setPosition(0);
                        leftNo = true;
                        stage++;
                    } else if (centerPurple) {
                        centerPin.setPosition(0);
                        centerNo = true;
                        stage++;
                    } else if (rightPurple) {
                        rightPin.setPosition(0);
                        rightNo = true;
                        stage++;
                    }
                } else if (stage == 2) {
                    if (leftPurple){
                        leftPin.setPosition(0);
                        leftNo = true;
                        stage++;
                    } else if (centerPurple) {
                        centerPin.setPosition(0);
                        centerNo = true;
                        stage++;
                    } else if (rightPurple) {
                        rightPin.setPosition(0);
                        rightNo = true;
                        stage++;
                    }
                }
            }


        }
    }
}
