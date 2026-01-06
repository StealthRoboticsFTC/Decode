package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Configurable
public class SystemTest extends LinearOpMode {

    public static boolean usePins = false;

    public static int turretTargetPosition = 0;


    public static int shooterTargetVelocity = 0;

    public static double flapPosition = 0.8;

    public static double intakePower = 0;

    public static double transferPower = 0;

    public static double leftPinPositon = 0.325;
    public static double centerPinPosition = 0.425;
    public static double rightPinPosition = 0.85;




    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDFController turretController = new PIDFController(new PIDFCoefficients(-0.000425, 0, -0.000015,-0.0000325));

        Servo flap = hardwareMap.get(Servo.class, "servo_sf");
        DcMotorEx shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "motor_sl");
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor_sr");
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDFController shooterController = new PIDFController(new PIDFCoefficients(0.005, 0, 0, 0.0005));

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "motor_im");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        CRServo transferLeft = hardwareMap.get(CRServo.class, "servo_tl");
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        CRServo transferRight = hardwareMap.get(CRServo.class, "servo_tr");

        Servo leftPin = hardwareMap.get(Servo.class,"servo_lp");
        Servo centerPin = hardwareMap.get(Servo.class,"servo_cp");
        Servo rightPin = hardwareMap.get(Servo.class,"servo_rp");

        ColorSensor leftColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cl");
        ColorSensor rightColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cr");
        ColorSensor centerColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cc");

        Servo leftLight = hardwareMap.get(Servo.class, "light_ll");
        Servo rightLight = hardwareMap.get(Servo.class, "light_rl");
        Servo centerLight = hardwareMap.get(Servo.class, "light_cl");


        waitForStart();
        while (!isStopRequested()){
            if (turretTargetPosition != 0){
                turretController.setTargetPosition(turretTargetPosition);
                turretController.updatePosition(turret.getCurrentPosition());
                turret.setPower(turretController.run());
                telemetry.addData("power", turretController.run());
                telemetry.addData("currentPosition", turret.getCurrentPosition());
                telemetry.addData("TargetPosition", turretTargetPosition);
                telemetry.update();
            }
            if (shooterTargetVelocity != 0){
                shooterController.updateFeedForwardInput(shooterTargetVelocity);
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
            if (intakePower != 0){
                intakeMotor.setPower(intakePower);


            }
            if (transferPower != 0){
                transferRight.setPower(transferPower);
                transferLeft.setPower(transferPower);

            }
            if (usePins){
                centerPin.setPosition(centerPinPosition);
                leftPin.setPosition(leftPinPositon);
                rightPin.setPosition(rightPinPosition);
            }

            if (leftColorSensor.blue()<100 &&leftColorSensor.red()<100 && leftColorSensor.green()<100){
                telemetry.addData("Left Color", "none");
                leftLight.setPosition(0.28);


            }
            else if(leftColorSensor.green() > leftColorSensor.blue() && leftColorSensor.green() > leftColorSensor.red()){
                telemetry.addData("Left Color", "Green");
                leftLight.setPosition(0.444);
            }
            else {
                telemetry.addData("Left Color", "Purple");
                leftLight.setPosition(0.722);

            }
            if (rightColorSensor.blue()<100 &&rightColorSensor.red()<100 && rightColorSensor.green()<100){
                telemetry.addData("Right Color", "none");
                rightLight.setPosition(0.28);

            }
            else if(rightColorSensor.green() > rightColorSensor.blue() && rightColorSensor.green() > rightColorSensor.red()){
                telemetry.addData("Right Color", "Green");
                rightLight.setPosition(0.444);
            }
            else {
                telemetry.addData("Right Color", "Purple");
                rightLight.setPosition(0.722);
            }
            if (centerColorSensor.blue()<100 &&centerColorSensor.red()<100 && centerColorSensor.green()<100){
                telemetry.addData("CenterColor", "none");
                centerLight.setPosition(0.28);

            }
            else if(centerColorSensor.green() > centerColorSensor.blue() && centerColorSensor.green() > centerColorSensor.red()){
                telemetry.addData("Center Color", "Green");
                centerLight.setPosition(0.444);


            }
            else {
                telemetry.addData("Center Color", "Purple");
                centerLight.setPosition(0.277);
            }

            telemetry.update();


        }

    }
}
