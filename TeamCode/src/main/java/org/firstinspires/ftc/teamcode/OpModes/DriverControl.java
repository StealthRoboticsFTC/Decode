package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Processor;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.IntakeBall;
import org.firstinspires.ftc.teamcode.common.command.OuttakeBall;
import org.firstinspires.ftc.teamcode.common.command.Reset;
import org.firstinspires.ftc.teamcode.common.command.ShootClose;
import org.firstinspires.ftc.teamcode.common.command.ShootFar;
import org.firstinspires.ftc.teamcode.common.command.ShootMedium;
import org.firstinspires.ftc.teamcode.common.controller.Button;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;
@TeleOp
public class DriverControl extends LinearOpMode {
    private Robot robot;

    private ButtonListener listener;

    private static Pose startingPose;

    private Processor processor;


    @Override
    public void runOpMode() throws InterruptedException {
        listener = new ButtonListener(gamepad1);
        robot = new Robot(hardwareMap);
        processor = new Processor();

        listener.addListener(Button.R_TRIGGER_DOWN, ()->{
            processor.override(new ShootMedium());
        });
        listener.addListener(Button.R_BUMPER_DOWN, ()->{
            processor.override(new ShootClose());
        });
        listener.addListener(Button.L_TRIGGER_DOWN, ()->{
            processor.override(new IntakeBall());
        });
        listener.addListener(Button.L_BUMPER_DOWN, ()->{
            processor.override(new OuttakeBall());
        });
        listener.addListener(Button.TRIANGLE_DOWN, ()->{
            processor.override(new ShootFar());
        });
        listener.addListener(Button.CROSS_DOWN, ()->{
            processor.override(new Reset());
        });

        robot.follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        robot.follower.update();
        waitForStart();
        robot.follower.startTeleOpDrive(true);

        while (!isStopRequested()){

            robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            robot.update();
            listener.update();
            processor.update(robot);
            telemetry.addData("atVelocity", robot.shooter.atTargetVelocity());
            telemetry.addData("velocity", robot.shooter.getVelocity());
            telemetry.addData("targetVelocity", robot.shooter.getTargetVelocity());
            telemetry.update();
        }
    }
}
