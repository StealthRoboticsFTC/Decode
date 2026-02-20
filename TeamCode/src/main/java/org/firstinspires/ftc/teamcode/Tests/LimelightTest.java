package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Configurable
public class LimelightTest extends LinearOpMode {
    public static double kp = 0.0475;
    public static double ki = 0;
    public static double kd = 0.00025;
    public static double kf = 0;

    public  static int targetVelocity = 0;
    public  static double flapPos = 0.7;

    public static double targetTx = -2.5;


    public static int pipeLine = 0;
    PIDFController turretController;
    PIDFController shooterController;


    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limeLight");
        DcMotor turret = hardwareMap.get(DcMotor.class, "motor_tm");
        DcMotorEx shooterLeft = hardwareMap.get(DcMotorEx.class, "motor_sl");
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "motor_sr");
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo flap = hardwareMap.get(Servo.class, "servo_sf");

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretController = new PIDFController(new PIDFCoefficients(kp,ki,kd,kf));
        shooterController = new PIDFController(new PIDFCoefficients(0.0075, 0, 0, 0.0006675));
        limelight3A.pipelineSwitch(pipeLine);
        waitForStart();
        limelight3A.start();
        while (!isStopRequested()){
            limelight3A.pipelineSwitch(pipeLine);
            turretController.setD(kd);
            turretController.setF(kf);
            turretController.setP(kp);
            turretController.setI(ki);


            LLResult result = limelight3A.getLatestResult();
            if (result != null && result.isValid() ){
                double x = result.getTy();
                double velocity = targetVelocity;


                turretController.updateFeedForwardInput(targetTx);
                turretController.setTargetPosition(targetTx);
                turretController.updatePosition(result.getTx());
                turret.setPower(turretController.run());
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("targetVelocity", velocity);
                shooterController.setTargetPosition(velocity);
                shooterController.updateFeedForwardInput(velocity);
                shooterController.updatePosition(shooterLeft.getVelocity());
                shooterRight.setPower(shooterController.run());
                shooterLeft.setPower(shooterController.run());
                flap.setPosition(flapPos);
            }
            else {
                turret.setPower(0);
                shooterRight.setPower(0.75);
                shooterLeft.setPower(0.75);
            }
            telemetry.addData("power", turretController.run());
            telemetry.addData("Velocity", shooterLeft.getVelocity());
            telemetry.update();
        }
    }
}
