package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Processor;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.Reset;
import org.firstinspires.ftc.teamcode.common.command.ShootFarRed;
@Autonomous

public class RedFarAuto extends LinearOpMode {
    Robot robot;
    Processor processor;
    ElapsedTime elapsedTime;
    private static int stage;
    private Pose startPose = new Pose(85, 12, Math.toRadians(270));
    private Pose shootPose = new Pose(85, 23, Math.toRadians(270));
    private Pose parkPose = new Pose(85, 34, Math.toRadians(270));

    private PathChain score, park;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        processor = new Processor();
        elapsedTime = new ElapsedTime();
        stage = 0;

        robot.follower.setStartingPose(startPose);

       score =  robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
               .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
               .build();
       park = robot.follower.pathBuilder()
               .addPath(new BezierLine(shootPose, parkPose))
               .setLinearHeadingInterpolation(shootPose.getHeading(), shootPose.getHeading())
               .build();

       waitForStart();
       while (!isStopRequested()){
           if (stage == 0){
               robot.follower.setMaxPower(0.25);
               processor.override(new ShootFarRed());
               robot.follower.followPath(score);
               elapsedTime.reset();
               stage++;
           } else if (stage == 1 && elapsedTime.milliseconds() > 7500){
               robot.follower.followPath(park);
               processor.override(new Reset());
               stage++;
           }
           telemetry.addData("stage", stage);
           telemetry.update();
           robot.update();
           processor.update(robot);

       }

    }
}
