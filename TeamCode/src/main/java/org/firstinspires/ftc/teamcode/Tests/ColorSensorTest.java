package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensor_cr");

        Servo leftLight = hardwareMap.get(Servo.class, "light_ll");
        Servo rightLight = hardwareMap.get(Servo.class, "light_rl");
        Servo centerLight = hardwareMap.get(Servo.class, "light_cl");
        waitForStart();
        while (!isStopRequested()){

            if (colorSensor.blue()<100 && colorSensor.red()<100 && colorSensor.green()<100){
                telemetry.addData("Color", "none");
                leftLight.setPosition(0.277);
                rightLight.setPosition(0.277);
                centerLight.setPosition(0.277);
            }
            else if(colorSensor.green() > colorSensor.blue() && colorSensor.green() > colorSensor.red()){
                telemetry.addData("Color", "Green");
                leftLight.setPosition(0.5);
                rightLight.setPosition(0.5);
                centerLight.setPosition(0.5);

            }
            else {
                telemetry.addData("Color", "Purple");
                leftLight.setPosition(0.722);
                rightLight.setPosition(0.722);
                centerLight.setPosition(0.722);
            }
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.update();
        }
    }
}
