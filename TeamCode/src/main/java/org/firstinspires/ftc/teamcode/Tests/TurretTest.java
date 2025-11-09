package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Configurable
@TeleOp
public class TurretTest extends LinearOpMode {
    public static double turretPower = 0;
    public static int turretPosition = 0;
    DcMotor turretMotor;
    AnalogInput turntableAbsoluteEncoder;
    @Override
    public void runOpMode() throws InterruptedException {
        turretMotor = hardwareMap.get(DcMotor.class, "motor_tm");
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turntableAbsoluteEncoder = hardwareMap.get(AnalogInput.class, "analog_tae");
        waitForStart();
        while (!isStopRequested()){
            double positon = turntableAbsoluteEncoder.getVoltage()/3.2*360;
            turretMotor.setTargetPosition(turretPosition);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(turretPower);
            telemetry.addData("TurretPosition", turretMotor.getCurrentPosition());
            telemetry.addData("TurntableAbsolutePosition", positon);
            telemetry.update();

        }

    }
}
