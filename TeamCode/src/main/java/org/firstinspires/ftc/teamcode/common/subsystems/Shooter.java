package org.firstinspires.ftc.teamcode.common.subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.Command;

public class Shooter {
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final Servo flap;

    private final static int closeVelocity = 915;
    private final static int mediumVelocity = 1100;
    private final static int farVelocity = 1385;

    private final static int reverse = -250;
    private final static int offVelocity = 0;

    private final static double flapPositionClose = 0.9;
    private final static double flapPositionMedium= 0.6675;
    private final static double flapPositionFar= 0.75;



    private final static double kp = 0.0075;
    private final static double ki = 0;
    private final static double kd = 0;
    private final static double kf = 0.0006675;
            ;

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

    public void shootMedium(){

        controller.setTargetPosition(mediumVelocity);
        flap.setPosition(flapPositionMedium);
    }

    public void shooterOff(){

        controller.setTargetPosition(offVelocity);
    }

    public void reverseShooter(){

        controller.setTargetPosition(reverse);
    }

    public void setTargetVelocity(double targetVelocity, double flapPos){
        flap.setPosition(flapPos);

        controller.setTargetPosition(targetVelocity);

    }

    public boolean atTargetVelocity(){
        return shooterLeft.getVelocity() > controller.getTargetPosition() - 50 && shooterLeft.getVelocity() < controller.getTargetPosition() + 50;
    }

    public double getVelocity(){
        return shooterLeft.getVelocity();
    }
    public double getTargetVelocity(){
        return controller.getTargetPosition();
    }

    public void update( double LimeLightTargetVelocity, double LimeLightFlapPos){
        if (Robot.useAutoAim){
            controller.setTargetPosition(LimeLightTargetVelocity);
            flap.setPosition(LimeLightFlapPos);
        } else if (!Robot.manul) {
            controller.setTargetPosition(0);
        }
        controller.updateFeedForwardInput(controller.getTargetPosition());
        controller.updatePosition(shooterLeft.getVelocity());
        shooterRight.setPower(controller.run());
        shooterLeft.setPower(controller.run());

    }


}
