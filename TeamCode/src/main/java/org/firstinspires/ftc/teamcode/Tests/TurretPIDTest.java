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
public class TurretPIDTest extends LinearOpMode {
    public static int turretTargetPosition = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    DcMotorEx turret;

    PIDFController controller;


    @Override
    public void runOpMode() throws InterruptedException {


        turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(new PIDFCoefficients(kp, ki, kd, kf));
        waitForStart();
        while (!isStopRequested()){
            controller.setD(kd);
            controller.setF(kf);
            controller.setP(kp);
            controller.setI(ki);
            controller.updateFeedForwardInput(turretTargetPosition);
            controller.setTargetPosition(turretTargetPosition);
            controller.updatePosition(turret.getCurrentPosition());
            turret.setPower(controller.run());
            telemetry.addData("power", controller.run());
            telemetry.addData("currentPosition", turret.getCurrentPosition());
            telemetry.addData("TargetPosition", turretTargetPosition);
            telemetry.update();
        }

    }
}
