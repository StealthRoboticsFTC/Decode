package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx intake;
    private final static double intakeOn = 1;

    private final static double outTake = -1;

    private final static double intakeOff= 0;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "motor_im");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnOnIntake(){
        intake.setPower(intakeOn);
    }

    public void turnOffIntake(){
        intake.setPower(intakeOff);
    }

    public void outtake(){intake.setPower(outTake);}
}
