package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp
@Configurable
public class odometryTurretTest extends LinearOpMode {
    private final Pose goalPos = new Pose(144, 144, Math.toRadians(45));
    public static double newX = 0;
    public static double newY = 0;

    public static double newHeading;
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0, Math.toRadians(270)));


        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDFController controller =  new PIDFController(new PIDFCoefficients(0.0525, 0, 0.00125, 0));

        waitForStart();

        while (!isStopRequested()){

            double turretAngle;
            if (turret.getCurrentPosition() != 0){
                 turretAngle = -360/((4096*((double) 70 /14))/ turret.getCurrentPosition());
            } else {
                turretAngle = 0;
            }
            Pose robotPos = follower.getPose();
            double gobleAngle = Math.atan2(goalPos.getY() - robotPos.getY(), goalPos.getX() - robotPos.getX());
            double turretHeading = robotPos.getHeading() + Math.PI;
            if (turretHeading > Math.PI) turretHeading -= 2*Math.PI;
            if (turretHeading < -Math.PI) turretHeading += 2 * Math.PI;
            double targetAngle = Math.toDegrees(gobleAngle - turretHeading) ;
            if (targetAngle>180) targetAngle -= 360;
            boolean atAngle;
            atAngle = turretAngle > targetAngle - 1 && turretAngle < targetAngle + 1;
            controller.setTargetPosition(targetAngle);
            controller.updatePosition(turretAngle);


            panelsTelemetry.addData("atAngle", atAngle);
            panelsTelemetry.addData("targetAngle", targetAngle);
            panelsTelemetry.addData("gobleAngle", Math.toDegrees(gobleAngle));
            panelsTelemetry.addData("turretAngle", turretAngle);
            panelsTelemetry.addData("turretPosition", turret.getCurrentPosition());
            panelsTelemetry.addData("turretHeading", Math.toDegrees(turretHeading));
            panelsTelemetry.addData("RobotAngle", Math.toDegrees(robotPos.getHeading()));
            panelsTelemetry.update();


            follower.update();
        }
    }
}
