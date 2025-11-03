package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    private final CRServo transferLeft;
    private final CRServo transferRight;
    private final static double transferOn = 1;
    private final static double transferOff= 0;

    public Transfer(HardwareMap hardwareMap){
        transferLeft = hardwareMap.get(CRServo.class, "servo_tl");
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        transferRight = hardwareMap.get(CRServo.class, "servo_tr");
    }

    public void turnOnTransfer(){
        transferLeft.setPower(transferOn);
        transferRight.setPower(transferOn);
    }

    public void turnOffIntake(){
        transferLeft.setPower(transferOff);
        transferRight.setPower(transferOff);
    }
}
