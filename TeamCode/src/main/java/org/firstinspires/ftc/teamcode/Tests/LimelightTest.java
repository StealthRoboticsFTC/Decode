package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Configurable
public class LimelightTest extends LinearOpMode {
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    PIDFController controller;
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limeLight");
        DcMotor turret = hardwareMap.get(DcMotor.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(new PIDFCoefficients(kp,ki,kd,kf));
        limelight3A.pipelineSwitch(0);
        waitForStart();
        limelight3A.start();
        while (!isStopRequested()){
            controller.setD(kd);
            controller.setF(kf);
            controller.setP(kp);
            controller.setI(ki);


            LLResult result = limelight3A.getLatestResult();
            if (result != null && result.isValid() ){
                turret.setPower(controller.run());
                controller.updateFeedForwardInput(0);
                controller.setTargetPosition(0);
                controller.updatePosition(result.getTx());
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
            }
            else {
                turret.setPower(0);
            }
            telemetry.addData("power", controller.run());
            telemetry.update();
        }
    }
}
