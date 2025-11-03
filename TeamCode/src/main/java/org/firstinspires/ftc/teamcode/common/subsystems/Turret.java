package org.firstinspires.ftc.teamcode.common.subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotorEx turret;
    private AnalogInput encoder;

    public final static int redClose = 3500;
    public final static int redFar = 500;
    public final static int blueClose = 1500;
    public final static int blueFar = 4500;

    public final static int zeroPositon = 300;

    private boolean zero = true;

    private final static double kp = 0;
    private final static double ki = 0;
    private final static double kd = 0;
    private final static double kf = 0;

    private final PIDFController controller;


    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoder = hardwareMap.get(AnalogInput.class, "analog_tae");

        PIDFCoefficients coefficients = new PIDFCoefficients(kp, ki, kd, kf);
        controller = new PIDFController(coefficients);
    }
    public void moveTurret(int position){
        controller.setTargetPosition(position);
    }

    public void update(){
        double absolutePositon = encoder.getVoltage()/3.2*360;

        if (zero && controller.getTargetPosition() == zeroPositon){
            controller.updatePosition(absolutePositon);
            if (absolutePositon > zeroPositon-5 && absolutePositon < zeroPositon+5 ){
                zero = false;
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }   else {
            controller.updatePosition(turret.getCurrentPosition());
        }
        turret.setPower(controller.run());

    }
}
