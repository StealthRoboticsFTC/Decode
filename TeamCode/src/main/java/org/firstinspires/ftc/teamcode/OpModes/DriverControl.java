package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.common.Processor;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.IntakeBall;
import org.firstinspires.ftc.teamcode.common.command.OuttakeBall;
import org.firstinspires.ftc.teamcode.common.command.Reset;
import org.firstinspires.ftc.teamcode.common.command.Shoot;
import org.firstinspires.ftc.teamcode.common.command.Sort;
import org.firstinspires.ftc.teamcode.common.controller.Button;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;
import org.firstinspires.ftc.teamcode.common.enums.Color;

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
            if (Robot.sort){
                processor.override(new Sort(21));
            } else {
                processor.override(new Shoot());
            }

        });
        listener.addListener(Button.R_BUMPER_DOWN, ()->{
            if (Robot.sort){
                Robot.sort = false;
            } else {
                Robot.sort = true;
            }
        });
        listener.addListener(Button.L_TRIGGER_DOWN, ()->{
            if (Robot.sort){
                processor.override(new IntakeBall(true));
            } else {
                processor.override(new IntakeBall(false));
            }

        });
        listener.addListener(Button.L_BUMPER_DOWN, ()->{
            processor.override(new OuttakeBall());
        });

        listener.addListener(Button.CROSS_DOWN, ()->{
            processor.override(new Reset());

        });

        Robot.color = Color.RED;
        Robot.sort = false;

        robot.follower.setStartingPose(new Pose(8, 5, Math.toRadians(90)));
        robot.follower.update();
        waitForStart();

        robot.follower.setMaxPower(1);
        robot.follower.startTeleOpDrive(true);

        while (!isStopRequested()){

            robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            robot.limelight.setPipLine(Robot.color);
            robot.update();
            listener.update();
            processor.update(robot);
            telemetry.addData("sort", Robot.sort);
            telemetry.addData("Colors", robot.colorSensors.getColors());
            telemetry.addData("velocity", robot.shooter.getVelocity());
            telemetry.addData("targetVelocity", robot.shooter.getTargetVelocity());
            telemetry.update();
        }
    }
}
