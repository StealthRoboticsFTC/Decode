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

    private final static int offVelocity = 0;

    private final static double kp = 0.0075;
    private final static double ki = 0;
    private final static double kd = 0;
    private final static double kf = 0.0005;
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



    public void shooterOff(){

        controller.setTargetPosition(offVelocity);
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

    public void update( double targetVelocity, double flapPos){
        controller.setTargetPosition(targetVelocity);
        controller.updateFeedForwardInput(controller.getTargetPosition());
        controller.updatePosition(shooterLeft.getVelocity());
        shooterRight.setPower(controller.run());
        shooterLeft.setPower(controller.run());
        flap.setPosition(flapPos);

    }


}
