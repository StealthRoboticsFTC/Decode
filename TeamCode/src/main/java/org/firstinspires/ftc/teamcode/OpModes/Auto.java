package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Processor;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;
import org.firstinspires.ftc.teamcode.common.enums.Color;
import org.firstinspires.ftc.teamcode.common.enums.Position;
import org.firstinspires.ftc.teamcode.common.menu.Menu;
import org.firstinspires.ftc.teamcode.common.menu.Screen;

import java.util.Arrays;
import java.util.List;

public class Auto extends LinearOpMode {
    Robot robot;
    Processor processor;
    ElapsedTime elapsedTime;
    private static int stage;
    private static final int shootTime = 3000;
    private final double heading = Math.toRadians(180);

    private final Pose farScore1Pose = new Pose(42,8,heading);
    private final Pose farScore2Pose = new Pose(60,25, heading);
    private final Pose farPickup1Pose = new Pose(9,8, heading);
    private final Pose farPickup2Pose = new Pose(15,36, heading);
    private final Pose farPickup2Control = new Pose(45,35, heading);
    private final Pose farPickup3Pose = new Pose(12,65, heading);
    private final Pose farPickup3Control = new Pose(60,55, heading);
    private final Pose farPickup4Pose = new Pose(9,25, heading);

    private final Pose farParkPose = new Pose(25,8, heading);



    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, grabPickup4, scorePickup4, grabPickup5, scorePickup5, park;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        processor = new Processor();
        elapsedTime = new ElapsedTime();
        stage = 0;


        Menu menu = new Menu(new ButtonListener(gamepad1), telemetry);
        menu.addScreen(new Screen("Color", Arrays.asList(Color.values())))
                .addScreen(new Screen<>("Position", Arrays.asList(Position.values())));

        menu.runBlocking();

        List<Object> options = menu.getSelections();
        Robot.color = (Color) options.get(0);
        Robot.position = (Position) options.get(1);
        waitForStart();

        if (Robot.color == Color.BLUE && Robot.position == Position.Far){
            robot.follower.setStartingPose(farScore1Pose);



            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose,farPickup1Pose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farPickup1Pose.getHeading())
                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup1Pose.getHeading(), farScore2Pose.getHeading())
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose, farPickup2Control,farPickup2Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup2Pose.getHeading())
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup2Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup2Pose.getHeading(), farScore2Pose.getHeading())
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose, farPickup3Control, farPickup3Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup3Pose.getHeading())
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup3Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup3Pose.getHeading(),farScore2Pose.getHeading())
                    .build();
            grabPickup4  = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore2Pose, farPickup4Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup4Pose.getHeading())
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup4Pose, farScore1Pose))
                    .setLinearHeadingInterpolation(farPickup4Pose.getHeading(), farScore1Pose.getHeading())
                    .build();
            grabPickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose, farPickup1Pose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farPickup1Pose.getHeading())
                    .build();

            scorePickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose, farScore1Pose))
                    .setLinearHeadingInterpolation(farPickup1Pose.getHeading(), farScore1Pose.getHeading())
                    .build();
            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose,farParkPose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farParkPose.getHeading())
                    .build();
        }
        else if (Robot.color == Color.RED && Robot.position == Position.Far){
            robot.follower.setStartingPose(farScore1Pose.mirror());



            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose.mirror(),farPickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farPickup1Pose.mirror().getHeading())
                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup1Pose.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose.mirror(), farPickup2Control.mirror(),farPickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup2Pose.mirror().getHeading())
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup2Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup2Pose.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose.mirror(), farPickup3Control.mirror(), farPickup3Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup3Pose.mirror().getHeading())
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup3Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup3Pose.mirror().getHeading(),farScore2Pose.mirror().getHeading())
                    .build();
            grabPickup4  = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore2Pose.mirror(), farPickup4Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup4Pose.mirror().getHeading())
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup4Pose.mirror(), farScore1Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup4Pose.mirror().getHeading(), farScore1Pose.mirror().getHeading())
                    .build();
            grabPickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose.mirror(), farPickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farPickup1Pose.mirror().getHeading())
                    .build();

            scorePickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose.mirror(), farScore1Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup1Pose.mirror().getHeading(), farScore1Pose.mirror().getHeading())
                    .build();
            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose.mirror(),farParkPose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farParkPose.mirror().getHeading())
                    .build();
        }

        robot.follower.setMaxPower(1);
        robot.limelight.setPipLine(Robot.color);
        while (!isStopRequested()){
            if (stage == 1){
                //shoot
                increment();
            } else if (stage == 2 && elapsedTime.milliseconds()>shootTime) {
                //intake
                robot.follower.followPath(grabPickup1);
                increment();
            } else if (stage == 3 && !robot.follower.isBusy()) {
                robot.follower.followPath(scorePickup1);
                increment();
            } else if (stage == 4 &&!robot.follower.isBusy()) {
                //shoot
                increment();
            } else if (stage == 5 && elapsedTime.milliseconds()>shootTime) {
                //intake
                robot.follower.followPath(grabPickup2);
                increment();
            } else if (stage == 6 && !robot.follower.isBusy()) {
                robot.follower.followPath(scorePickup2);
                increment();
            } else if (stage == 7 &&!robot.follower.isBusy()) {
                //shoot
                increment();
            } else if (stage == 8 && elapsedTime.milliseconds()>shootTime) {
                //intake
                robot.follower.followPath(grabPickup3);
                increment();
            } else if (stage == 9 && !robot.follower.isBusy()) {
                robot.follower.followPath(scorePickup3);
                increment();
            } else if (stage == 10 &&!robot.follower.isBusy()) {
                //shoot
                increment();
            } else if (stage == 11 && elapsedTime.milliseconds()>shootTime) {
                //intake
                robot.follower.followPath(grabPickup4);
                increment();
            } else if (stage == 12 && !robot.follower.isBusy()) {
                robot.follower.followPath(scorePickup4);
                increment();
            } else if (stage == 13 &&!robot.follower.isBusy()) {
                //shoot
                increment();
            }else if (stage == 14 && elapsedTime.milliseconds()>shootTime) {
                //intake
                robot.follower.followPath(grabPickup5);
                increment();
            } else if (stage == 15 && !robot.follower.isBusy()) {
                robot.follower.followPath(scorePickup5);
                increment();
            } else if (stage == 16 &&!robot.follower.isBusy()) {
                //shoot
                increment();
            } else if (stage == 17 && !robot.follower.isBusy()) {
                //reset
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

