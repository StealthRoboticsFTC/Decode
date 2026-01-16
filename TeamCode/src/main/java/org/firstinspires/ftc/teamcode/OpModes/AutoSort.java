package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.common.Processor;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.IntakeBall;
import org.firstinspires.ftc.teamcode.common.command.Sort;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;
import org.firstinspires.ftc.teamcode.common.enums.Color;
import org.firstinspires.ftc.teamcode.common.enums.Position;
import org.firstinspires.ftc.teamcode.common.menu.Menu;
import org.firstinspires.ftc.teamcode.common.menu.Screen;

import java.util.Arrays;
import java.util.List;

public class AutoSort extends LinearOpMode {
    Robot robot;
    Processor processor;
    ElapsedTime elapsedTime;
    private static int stage;
    private static final int shootTime = 4000;

    private final Pose startPose = new Pose(120, 129, Math.toRadians(315));
    private final Pose Score1Pose = new Pose(90,99 ,Math.toRadians(350));
    private final Pose Score2Pose = new Pose(80,89, Math.toRadians(300));
    private final Pose pickup1Pose = new Pose(123,85, Math.toRadians(300));
    private final Pose pickup2Pose = new Pose(125,60, Math.toRadians(300));

    private final Pose parkPose = new Pose(125,70, Math.toRadians(270));



    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2,  park;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        processor = new Processor();
        elapsedTime = new ElapsedTime();
        stage = 0;


        Menu menu = new Menu(new ButtonListener(gamepad1), telemetry);
        menu.addScreen(new Screen("Color", Arrays.asList(Color.values())));


        menu.runBlocking();

        List<Object> options = menu.getSelections();
        Robot.color = (Color) options.get(0);
        robot.pins.closeAllPin();
        waitForStart();

        if (Robot.color == Color.BLUE){
            robot.follower.setStartingPose(startPose.mirror());

            scorePreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(startPose.mirror(), Score1Pose.mirror()))
                    .setLinearHeadingInterpolation(startPose.mirror().getHeading(), Score1Pose.mirror().getHeading())
                    .build();


            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(Score1Pose.mirror(),pickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(Score1Pose.mirror().getHeading(), pickup1Pose.mirror().getHeading())

                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(pickup1Pose.mirror(), Score2Pose.mirror()))
                    .setLinearHeadingInterpolation(pickup1Pose.mirror().getHeading(), Score2Pose.mirror().getHeading())
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(Score2Pose.mirror(), pickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(Score2Pose.mirror().getHeading(), pickup2Pose.mirror().getHeading())
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose.mirror(), Score2Pose.mirror()))
                    .setLinearHeadingInterpolation(pickup2Pose.mirror().getHeading(), Score2Pose.mirror().getHeading())
                    .build();

            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(Score2Pose.mirror(),parkPose.mirror()))
                    .setLinearHeadingInterpolation(Score1Pose.mirror().getHeading(), parkPose.mirror().getHeading())
                    .build();
        }
        else if (Robot.color == Color.RED ){
            robot.follower.setStartingPose(startPose);

            scorePreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(startPose, Score1Pose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), Score1Pose.getHeading())
                    .build();


            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(Score1Pose,pickup1Pose))
                    .setLinearHeadingInterpolation(Score1Pose.getHeading(), pickup1Pose.getHeading())

                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, Score2Pose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), Score2Pose.getHeading())
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(Score2Pose, pickup2Pose))
                    .setLinearHeadingInterpolation(Score2Pose.getHeading(), pickup2Pose.getHeading())
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, Score2Pose))
                    .setLinearHeadingInterpolation(pickup2Pose.getHeading(), Score2Pose.getHeading())
                    .build();

            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(Score2Pose,parkPose))
                    .setLinearHeadingInterpolation(Score1Pose.getHeading(), parkPose.getHeading())
                    .build();
        }

        robot.follower.setMaxPower(1);
        Robot.sort = true;
        Robot.useAutoAim = false;
        robot.limelight.setPipLine(null);
        while (!isStopRequested()){
            if (stage == 1){
                processor.override(new IntakeBall(true));
                if (Robot.color == Color.RED){
                    robot.setTurretAngle(120);
                } else {
                    robot.setTurretAngle(-120);
                }
                increment();
            } else if (stage == 2) {
                robot.follower.followPath(scorePreload);
                increment();
            } else if (stage == 3 && !robot.follower.isBusy()) {
                Robot.motif = robot.limelight.getMotif();
                Robot.useAutoAim = true;
                robot.limelight.setPipLine(Robot.color);
                increment();
            } else if (stage == 4 && elapsedTime.milliseconds() > 1000) {
                processor.override(new Sort(Robot.motif));
                increment();
            } else if (stage == 5 && elapsedTime.milliseconds()>shootTime) {
                processor.override(new IntakeBall(true));
                robot.follower.followPath(grabPickup1);
                increment();
            } else if (stage == 6 && !robot.follower.isBusy()) {
                robot.follower.followPath(scorePickup1);
                increment();
            } else if (stage == 7 &&!robot.follower.isBusy()) {
                processor.override(new Sort(Robot.motif));
                increment();
            } else if (stage == 8 && elapsedTime.milliseconds()>shootTime) {
                processor.override(new IntakeBall(true));
                robot.follower.followPath(grabPickup2);
                increment();
            } else if (stage == 9 && !robot.follower.isBusy()) {
                robot.follower.followPath(scorePickup2);
                increment();
            } else if (stage == 10 &&!robot.follower.isBusy()) {
                processor.override(new Sort(Robot.motif));
                increment();
            } else if (stage == 11 && elapsedTime.milliseconds()>shootTime) {

                robot.follower.followPath(park);
                increment();

            }
            telemetry.addData("stage", stage);
            telemetry.update();
            robot.update();
            processor.update(robot);
        }


    }
    private void increment(){
        stage++;
        elapsedTime.reset();
    }
}
