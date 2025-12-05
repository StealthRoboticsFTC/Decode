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
    private final static double reverseTransfer= -0.75;

    public Transfer(HardwareMap hardwareMap){
        transferLeft = hardwareMap.get(CRServo.class, "servo_tl");
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        transferRight = hardwareMap.get(CRServo.class, "servo_tr");
    }

    public void turnOnTransfer(){
        transferLeft.setPower(transferOn);
        transferRight.setPower(transferOn);
    }

    public void turnOffTransfer(){
        transferLeft.setPower(transferOff);
        transferRight.setPower(transferOff);
    }
    public void reverseTransfer(){
        transferLeft.setPower(reverseTransfer);
        transferRight.setPower(reverseTransfer);
    }
}
