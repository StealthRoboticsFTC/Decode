package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
        follower.setStartingPose(new Pose(8, 5, Math.toRadians(90)));


        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limeLight");
        PIDFController controller = new PIDFController(new PIDFCoefficients(0.025 , 0, 0.002,0.04 ));

        waitForStart();
        limelight3A.pipelineSwitch(0);
        limelight3A.start();


        while (!isStopRequested()) {

            double targetAngle;
            double turretAngle;
            boolean useVision;
            if (turret.getCurrentPosition() != 0) {
                turretAngle = -360 / ((4096 * ((double) 70 / 20)) / turret.getCurrentPosition());
            } else {
                turretAngle = 0;
            }



            LLResult result = limelight3A.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                targetAngle = turretAngle - tx;
                useVision = true;
            } else {

                Pose robotPos = follower.getPose();
                double gobleAngle = Math.atan2(goalPos.getY() - robotPos.getY(), goalPos.getX() - robotPos.getX());
                double turretHeading = robotPos.getHeading() + Math.PI;
                if (turretHeading > Math.PI) turretHeading -= 2 * Math.PI;
                if (turretHeading < -Math.PI) turretHeading += 2 * Math.PI;
                targetAngle = Math.toDegrees(gobleAngle - turretHeading);
                if (targetAngle > 180) targetAngle -= 360;
                useVision = false;
            }
            boolean atAngle = turretAngle > targetAngle - 1 && turretAngle < targetAngle + 1;

            if (Math.abs(targetAngle) < 125 && !atAngle) {
                controller.setTargetPosition(targetAngle);
                controller.updatePosition(turretAngle);
                turret.setPower(controller.run());
            } else turret.setPower(0);

            panelsTelemetry.addData("atAngle", atAngle);
            panelsTelemetry.addData("targetAngle", targetAngle);

            panelsTelemetry.addData("turretAngle", turretAngle);
            panelsTelemetry.addData("turretPosition", turret.getCurrentPosition());
            panelsTelemetry.addData("useVision", useVision);
            panelsTelemetry.update();
            limelight3A.updateRobotOrientation(Math.toDegrees(follower.getHeading()));


            follower.update();
        }
    }
}








