package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo superServoLeft= hardwareMap.get(CRServo.class, "servo_tl");
        CRServo superServoRight= hardwareMap.get(CRServo.class, "servo_tr");
        waitForStart();
        while (!isStopRequested()){
            superServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            superServoLeft.setPower(1);     
            superServoRight.setPower(1);
            telemetry.addData("Left", superServoLeft.getPortNumber());
            telemetry.addData("right", superServoRight.getPortNumber());
            telemetry.update();
        }

    }
}
