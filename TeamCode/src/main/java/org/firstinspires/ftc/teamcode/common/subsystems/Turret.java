package org.firstinspires.ftc.teamcode.common.subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.CustomPIDController;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.enums.Color;

public class Turret {
    private DcMotorEx turret;

    private final static int redClose = 2000;
    private final static int redFar = 1500;
    private final static int blueClose = -2000;
    private final static int blueFar = -1500;

    private final static int reset = 0;

    private final static double k = -360. / (4096. * (70. / 20.));


    private final CustomPIDController controller;
    double turretAngle;



    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new CustomPIDController();
        controller.setkD(0.0015);
        controller.setkP(4);
        controller.setkI(0.007);
        controller.setkF(0.001);
        controller.setkS(0.05);

    }




    public void update(double targetAngle){

        boolean atAngle = Math.abs(turretAngle - targetAngle) <= 1;

        if (Math.abs(targetAngle) < 125 && !atAngle) {
            controller.setTarget(targetAngle);
            double power = Math.clamp(controller.update(turretAngle, k * turret.getVelocity()), -0.65, 0.65);
            turret.setPower(power);
        } else {
            turret.setPower(0);
            controller.skip();
        }

    }

    public double getTurretAngle(){
        turretAngle = k * turret.getCurrentPosition();
        return turretAngle;
    }

}
