package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDFController(new PIDFCoefficients(kp, ki, kd, kf));
        waitForStart();
        while (!isStopRequested()){
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
