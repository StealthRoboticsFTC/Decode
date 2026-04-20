package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo transferLeft = hardwareMap.get(CRServo.class, "servo_tl");
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo transferRight = hardwareMap.get(CRServo.class, "servo_tr");
        waitForStart();
        while (!isStopRequested()){

            transferRight.setPower(1);
            transferLeft.setPower(1);


            telemetry.update();
        }

    }
}
