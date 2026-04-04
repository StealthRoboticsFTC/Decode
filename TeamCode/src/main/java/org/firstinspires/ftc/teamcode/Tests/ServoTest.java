package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo superServoLeftFront = hardwareMap.get(CRServo.class, "servo_tlf");
        superServoLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        CRServo superServoRightFront = hardwareMap.get(CRServo.class, "servo_trf");
        CRServo superServoLeftBack = hardwareMap.get(CRServo.class, "servo_tlb");
        superServoLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        CRServo superServoRightBack = hardwareMap.get(CRServo.class, "servo_trb");
        waitForStart();
        while (!isStopRequested()){

            superServoRightFront.setPower(1);
            superServoLeftBack.setPower(1);
            superServoLeftFront.setPower(1);
            superServoRightBack.setPower(1);

            telemetry.update();
        }

    }
}
