package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo superServoLeft= hardwareMap.get(CRServo.class, "SuperServoLeft");
        CRServo superServoRight= hardwareMap.get(CRServo.class, "SuperServoRight");
        waitForStart();
        while (!isStopRequested()){
            superServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            superServoLeft.setPower(1);     
            superServoRight.setPower(1);
        }

    }
}
