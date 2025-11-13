package org.firstinspires.ftc.teamcode.common.subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotorEx turret;

    private final static int redClose = 2000;
    private final static int redFar = 1000;
    private final static int blueClose = -2000;
    private final static int blueFar = -1000;

    private final static int reset = 0;
    private int turretTargetPosition = 0;





    private final static int zeroPositon = 300;

    private boolean zero = true;

    private final static double kp = -0.000425;
    private final static double ki = 0;
    private final static double kd = -0.000015;
    private final static double kf = -0.0000325;

    private final PIDFController controller;


    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        PIDFCoefficients coefficients = new PIDFCoefficients(kp, ki, kd, kf);
        controller = new PIDFController(coefficients);
    }
    public void moveTurretRedAuto(){
        controller.setTargetPosition(redClose);
    }
    public void moveTurretBlueAuto(){
        controller.setTargetPosition(blueClose);
    }

    public void  moveTurretRedFar(){
        controller.setTargetPosition(redFar);
    }
    public void turretReset(){
        controller.setTargetPosition(reset);
    }


    public void update(){
        controller.updateFeedForwardInput(turretTargetPosition);

        controller.updatePosition(turret.getCurrentPosition());
        turret.setPower(controller.run());


    }
}
