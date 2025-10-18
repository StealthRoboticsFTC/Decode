package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        Servo light = hardwareMap.get(Servo.class, "light");
        waitForStart();
        while (!isStopRequested()){

            if (colorSensor.blue()<100 && colorSensor.red()<100 && colorSensor.green()<100){
                telemetry.addData("Color", "none");
                light.setPosition(0.2875);
            }
            else if(colorSensor.green() > colorSensor.blue() && colorSensor.green() > colorSensor.red()){
                telemetry.addData("Color", "Green");
                light.setPosition(0.5);

            }
            else {
                telemetry.addData("Color", "Purple");
                light.setPosition(0.722);
            }
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.update();
        }
    }
}
