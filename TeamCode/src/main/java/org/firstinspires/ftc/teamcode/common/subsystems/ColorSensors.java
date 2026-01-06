package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.enums.BallColors;

import java.util.ArrayList;
import java.util.List;

public class ColorSensors {

    ColorSensor[] colorSensors;


    public ColorSensors(HardwareMap hardwareMap){
        ColorSensor leftColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cl");
        ColorSensor rightColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cr");
        ColorSensor centerColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cc");
        colorSensors = new ColorSensor[]{leftColorSensor, centerColorSensor, rightColorSensor};
    }
    public List<BallColors> getColors() {
        List<BallColors> ballColors = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            if (colorSensors[i].blue() < 100 && colorSensors[i].red() < 100 && colorSensors[i].green() < 100) {
                ballColors.add(i, BallColors.None);

            } else if (colorSensors[i].green() > colorSensors[i].blue() && colorSensors[i].green() > colorSensors[i].red()) {
                ballColors.add(i, BallColors.Green);
            } else {
                ballColors.add(i, BallColors.Purple);


            }
        }
        return ballColors;
    }
}
