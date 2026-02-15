package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.enums.BallColors;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ColorSensors {


    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;


    public ColorSensors(HardwareMap hardwareMap){
        leftColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cl");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cr");
    }
    public List<BallColors> getColors() {
        List<BallColors> ballColors = new ArrayList<>(Arrays.asList(BallColors.None, BallColors.None, BallColors.None));

        if (leftColorSensor.blue() < 100 && leftColorSensor.red() < 100 && leftColorSensor.green() < 100) {
            ballColors.set(0, BallColors.None);

        } else if (leftColorSensor.green() > leftColorSensor.blue() && leftColorSensor.green() > leftColorSensor.red()) {
            ballColors.set(0, BallColors.Green);
        } else {
            ballColors.set(0, BallColors.Purple);
        }

        if (rightColorSensor.blue() < 100 && rightColorSensor.red() < 100 && rightColorSensor.green() < 100) {
            ballColors.set(2, BallColors.None);

        } else if (rightColorSensor.green() > rightColorSensor.blue() && rightColorSensor.green() > rightColorSensor.red()) {
            ballColors.set(2, BallColors.Green);
        } else {
            ballColors.set(2, BallColors.Purple);
        }

        if (ballColors.get(0) == BallColors.None || ballColors.get(2) == BallColors.None){
            ballColors.set(1, BallColors.None);
        } else if (ballColors.get(0) == BallColors.Purple && ballColors.get(2) == BallColors.Purple) {
            ballColors.set(1, BallColors.Green);
        } else {
            ballColors.set(1, BallColors.Purple);
        }


        return ballColors;
    }

    public Boolean ballsInIntake(){
        return leftColorSensor.blue() >= 90 && leftColorSensor.red() >= 90 && leftColorSensor.green() >= 90 && rightColorSensor.blue() >= 90 && rightColorSensor.red() >= 90 && rightColorSensor.green() >= 90;
    }


}
