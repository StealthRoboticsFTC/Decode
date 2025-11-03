package org.firstinspires.ftc.teamcode.common.subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.command.Command;

public class Shooter {
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final Servo flap;

    private final static int closeVelocity = 1200;
    private final static int farVelocity = 1440;

    private final static double flapPositionClose= 0.5;
    private final static double flapPositionFar= 0.75;

    private final static double kp = 0;
    private final static double ki = 0;
    private final static double kd = 0;
    private final static double kf = 0;

    private final PIDFController controller;

    public Shooter(HardwareMap hardwareMap){
        shooterLeft = hardwareMap.get(DcMotorEx.class, "motor_sl");
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterRight = hardwareMap.get(DcMotorEx.class, "motor_sr");
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flap = hardwareMap.get(Servo.class, "servo_sf");

        PIDFCoefficients coefficients = new PIDFCoefficients(kp, ki, kd, kf);
        controller = new PIDFController(coefficients);

    }

    public void shootFar(){
        controller.setTargetPosition(farVelocity);
        flap.setPosition(flapPositionFar);
    }
    public void shootClose(){

        controller.setTargetPosition(closeVelocity);
        flap.setPosition(flapPositionClose);
    }

    public void update(){
        controller.updatePosition(shooterLeft.getVelocity());
        shooterRight.setPower(controller.run());
        shooterLeft.setPower(controller.run());

    }


}
