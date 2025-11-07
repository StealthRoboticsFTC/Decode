package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Configurable
public class SystemTest extends LinearOpMode {
    public static boolean useTurret = false;
    public static boolean useShooter = false;
    public static boolean useIntake = false;
    public static boolean useTransfer = false;
    public static boolean usePins = false;

    public static int turretTargetPosition = 0;
    public static double turretKp = 0;
    public static double turretKi = 0;
    public static double turretKd = 0;
    public static double turretKf = 0;

    public static int shooterTargetVelocity = 0;
    public static double shooterKp = 0;
    public static double shooterKi = 0;
    public static double shooterKd = 0;
    public static double shooerKf = 0;
    public static double flapPosition = 0.5;

    DcMotorEx shooterMotorLeft;
    DcMotorEx shooterMotorRight;
    Servo flap;
    PIDFController shooterController;

    DcMotorEx turret;
    PIDFController turretController;

    public static double intakePower = 0;
    DcMotor intakeMotor;

    CRServo transferLeft;
    CRServo transferRight;

    public static double transferPower = 0;

    Servo leftPin;
    Servo rightPin;
    Servo centerPin;

    public static double leftPinPositon = 0.25;
    public static double centerPinPosition = 0.5;
    public static double rightPinPosition = 0.75;




    @Override
    public void runOpMode() throws InterruptedException {


        turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretController = new PIDFController(new PIDFCoefficients(turretKp, turretKi, turretKd, turretKf));

        flap = hardwareMap.get(Servo.class, "servo_sf");
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "motor_sl");
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor_sr");
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterController = new PIDFController(new PIDFCoefficients(shooterKp, shooterKi, shooterKd, shooerKf));

        intakeMotor = hardwareMap.get(DcMotor.class, "motor_im");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        transferLeft = hardwareMap.get(CRServo.class, "servo_tl");
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRight = hardwareMap.get(CRServo.class, "servo_tr");

        leftPin = hardwareMap.get(Servo.class,"servo_lp");
        centerPin = hardwareMap.get(Servo.class,"servo_cp");
        rightPin = hardwareMap.get(Servo.class,"servo_rp");

        waitForStart();
        while (!isStopRequested()){
            if (useTurret){
                turretController.setTargetPosition(turretTargetPosition);
                turretController.updatePosition(turret.getCurrentPosition());
                turret.setPower(turretController.run());
                telemetry.addData("power", turretController.run());
                telemetry.addData("currentPosition", turret.getCurrentPosition());
                telemetry.addData("TargetPosition", turretTargetPosition);
                telemetry.update();
            }
            if (useShooter){
                shooterController.setTargetPosition(shooterTargetVelocity);
                shooterController.updatePosition(shooterMotorLeft.getVelocity());
                shooterMotorRight.setPower(shooterController.run());
                shooterMotorLeft.setPower(shooterController.run());
                flap.setPosition(flapPosition);
                telemetry.addData("TargetVelocity", shooterTargetVelocity);
                telemetry.addData("CurrentVelocity", shooterMotorLeft.getVelocity());
                telemetry.addData("Power", shooterController.run());
                telemetry.update();
            }
            if (useIntake){
                intakeMotor.setPower(intakePower);
            }
            if (useTransfer){
                transferRight.setPower(transferPower);
                transferLeft.setPower(transferPower);
            }
            if (usePins){
                centerPin.setPosition(centerPinPosition);
                leftPin.setPosition(leftPinPositon);
                rightPin.setPosition(rightPinPosition);
            }


        }

    }
}
