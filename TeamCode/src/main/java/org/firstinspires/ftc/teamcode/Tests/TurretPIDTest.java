package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Configurable
public class TurretPIDTest extends LinearOpMode {
    public static int turretTargetAngle = 0;
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
        controller = new PIDFController(new PIDFCoefficients(kp, ki, kd, kf));
        waitForStart();
        while (!isStopRequested()){
            double currentAngle;
            if (turret.getCurrentPosition() != 0){
                currentAngle = -360/((4096*((double) 70 /14))/ turret.getCurrentPosition());
            } else {
                currentAngle = 0;
            }
            controller.setD(kd);
            controller.setF(kf);
            controller.setP(kp);
            controller.setI(ki);
            controller.updateFeedForwardInput(turretTargetAngle);
            controller.setTargetPosition(turretTargetAngle);
            controller.updatePosition(currentAngle);
            turret.setPower(controller.run());
            telemetry.addData("power", controller.run());
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("TargetAngle", turretTargetAngle);
            telemetry.update();
        }

    }
}
