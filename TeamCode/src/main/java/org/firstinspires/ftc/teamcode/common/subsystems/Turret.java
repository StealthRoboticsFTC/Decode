package org.firstinspires.ftc.teamcode.common.subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.enums.Color;

public class Turret {
    private DcMotorEx turret;

    private final static int redClose = 2000;
    private final static int redFar = 1500;
    private final static int blueClose = -2000;
    private final static int blueFar = -1500;

    private final static int reset = 0;


    private final PIDFController encoderController;
    private final PIDFController limeLightController;


    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderController = new PIDFController(new PIDFCoefficients(-0.000425, 0, -0.000015,-0.0000325));
        limeLightController = new PIDFController(new PIDFCoefficients(0.025, 0, 0.00035, 0));
    }
    public void moveTurretRedAuto(){
        encoderController.setTargetPosition(redClose);
    }
    public void moveTurretBlueAuto(){
        encoderController.setTargetPosition(blueClose);
    }

    public void  moveTurretRedFar(){
        encoderController.setTargetPosition(redFar);
    }
    public void  moveTurretBlueFar(){
        encoderController.setTargetPosition(blueFar);
    }
    public void turretReset(){
        encoderController.setTargetPosition(reset);
    }


    public void update(double CurrentLimelightPos){

        turret.setPower(0);

    }
}
