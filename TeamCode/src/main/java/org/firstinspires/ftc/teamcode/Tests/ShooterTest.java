package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
@Configurable
public class ShooterTest extends LinearOpMode {
    public static double shooterPower = 0;
    DcMotorEx shooterMotorLeft;
    DcMotorEx shooterMotorRight;
    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "motor_sl");
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor_sr");
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        while (!isStopRequested()){
            shooterMotorLeft.setPower(shooterPower);
            shooterMotorRight.setPower(shooterPower);
            telemetry.addData("velocity", shooterMotorLeft.getVelocity());
            telemetry.update();
        }

    }
}
