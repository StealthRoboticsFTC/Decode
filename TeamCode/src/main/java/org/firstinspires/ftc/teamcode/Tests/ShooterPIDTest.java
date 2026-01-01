package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Configurable
public class ShooterPIDTest extends LinearOpMode {
    public static int shooterTargetVelocity = 0;
    public static double kp = 0.005;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.0005;
    public static double servoPosition = 0.8;
    DcMotorEx shooterMotorLeft;
    DcMotorEx shooterMotorRight;
    Servo flap;
    PIDFController controller;


    @Override
    public void runOpMode() throws InterruptedException {
        flap = hardwareMap.get(Servo.class, "servo_sf");

        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "motor_sl");
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor_sr");
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDFController(new PIDFCoefficients(kp, ki, kd, kf));
        waitForStart();
        while (!isStopRequested()){
            flap.setPosition(servoPosition);
            controller.setD(kd);
            controller.setF(kf);
            controller.setP(kp);
            controller.setI(ki);
            controller.updateFeedForwardInput(shooterTargetVelocity);
            controller.setTargetPosition(shooterTargetVelocity);
            controller.updatePosition(shooterMotorLeft.getVelocity());
            shooterMotorRight.setPower(controller.run());
            shooterMotorLeft.setPower(controller.run());

            telemetry.addData("TargetVelocity", shooterTargetVelocity);
            telemetry.addData("CurrentVelocity", shooterMotorLeft.getVelocity());
            telemetry.addData("Power", controller.run());
            telemetry.update();
        }


    }
}
