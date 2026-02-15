package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.enums.BallColors;

import java.util.List;

public class Lights {
    Servo[] lights;

    public Lights(HardwareMap hardwareMap){
        Servo leftLight = hardwareMap.get(Servo.class, "light_ll");
        Servo centerLight = hardwareMap.get(Servo.class, "light_cl");
        Servo rightLight = hardwareMap.get(Servo.class, "light_rl");
        lights = new Servo[]{leftLight,centerLight,rightLight};
    }
    public void setLights(List<BallColors> ballColors){
        for (int i = 0; i < 3; i++) {
            if (ballColors.get(i) == BallColors.Purple){
                lights[i].setPosition(0.722);
            } else if (ballColors.get(i) == BallColors.Green) {
                lights[i].setPosition(0.444);
            } else {
                lights[i].setPosition(0.28);
            }
        }
        lights[1].setPosition(0.28);

    }

    public void setToDefault(){
        for (int i = 0; i < 3; i++) {
            lights[i].setPosition(0.28);
        }
    }

}
