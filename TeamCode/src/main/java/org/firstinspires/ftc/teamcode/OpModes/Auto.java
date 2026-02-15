package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Processor;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.IntakeBall;
import org.firstinspires.ftc.teamcode.common.command.IntakeTransfer;
import org.firstinspires.ftc.teamcode.common.command.Shoot;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;
import org.firstinspires.ftc.teamcode.common.enums.Color;
import org.firstinspires.ftc.teamcode.common.enums.Position;
import org.firstinspires.ftc.teamcode.common.menu.Menu;
import org.firstinspires.ftc.teamcode.common.menu.Screen;

import java.util.Arrays;
import java.util.List;
@Autonomous
public class Auto extends LinearOpMode {
    Robot robot;
    Processor processor;
    ElapsedTime elapsedTime;
    private static int stage;
    private static final int shootTime = 1500;
    private final double heading = Math.toRadians(180);
    private final Pose start = new Pose(38,8, heading);

    private final Pose farScore1Pose = new Pose(40,8,heading);
    private final Pose farScore2Pose = new Pose(56,25, heading);


    private final Pose farPickup1Pose = new Pose(16,8, heading);
    private final Pose farPickup2Pose = new Pose(16,36, heading);
    private final Pose farPickup2Control = new Pose(45,35, heading);
    private final Pose farPickup3Pose = new Pose(16,66, heading);
    private final Pose farPickup3Control = new Pose(60,60, heading);
    private final Pose farPickup4Pose = new Pose(16,25, heading);

    private final Pose farParkPose = new Pose(30,8, heading);


    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, grabPickup4, scorePickup4, grabPickup5, scorePickup5, park;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        processor = new Processor();
        elapsedTime = new ElapsedTime();
        stage = 1;


       /* Menu menu = new Menu(new ButtonListener(gamepad1), telemetry);
        menu.addScreen(new Screen("Color", Arrays.asList(Color.values())))
                .addScreen(new Screen<>("Position", Arrays.asList(Position.values())));

        menu.runBlocking(); &

        List<Object> options = menu.getSelections();
        Robot.color = (Color) options.get(0);
        Robot.position = (Position) options.get(1);*/
        Robot.color = Color.RED;
        Robot.position = Position.Far;
        robot.pins.preload();
        robot.lifter.liftDown();
        waitForStart();

        if (Robot.color == Color.BLUE && Robot.position == Position.Far){
            robot.follower.setStartingPose(start);



            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(start,farPickup1Pose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farPickup1Pose.getHeading())
                    .setNoDeceleration()

                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup1Pose.getHeading(), farScore2Pose.getHeading())
                    .setBrakingStrength(2.5)
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose, farPickup2Control,farPickup2Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup2Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup2Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup2Pose.getHeading(), farScore2Pose.getHeading())
                    .setBrakingStart(2.5)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose, farPickup3Control, farPickup3Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup3Pose.getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup3Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup3Pose.getHeading(),farScore2Pose.getHeading())
                    .setBrakingStrength(2.5)
                    .build();
            grabPickup4  = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore2Pose, farPickup4Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup4Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup4Pose, farScore1Pose))
                    .setLinearHeadingInterpolation(farPickup4Pose.getHeading(), farScore1Pose.getHeading())
                    .setBrakingStrength(2.5)
                    .build();
            grabPickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose, farPickup1Pose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farPickup1Pose.getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose, farScore1Pose))
                    .setLinearHeadingInterpolation(farPickup1Pose.getHeading(), farScore1Pose.getHeading())
                    .setBrakingStrength(2.5)
                    .build();
            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose,farParkPose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farParkPose.getHeading())
                    .build();
        }
        else if (Robot.color == Color.RED && Robot.position == Position.Far){
            robot.follower.setStartingPose(start.mirror());



            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(start.mirror(),farPickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farPickup1Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup1Pose.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(2.5)
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose.mirror(), farPickup2Control.mirror(),farPickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup2Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup2Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup2Pose.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(2.5)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose.mirror(), farPickup3Control.mirror(), farPickup3Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup3Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup3Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup3Pose.mirror().getHeading(),farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(2.5)
                    .build();
            grabPickup4  = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore2Pose.mirror(), farPickup4Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup4Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup4Pose.mirror(), farScore1Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup4Pose.mirror().getHeading(), farScore1Pose.mirror().getHeading())
                    .setBrakingStrength(2.5)
                    .build();
            grabPickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose.mirror(), farPickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farPickup1Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose.mirror(), farScore1Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup1Pose.mirror().getHeading(), farScore1Pose.mirror().getHeading())
                    .setBrakingStrength(2.5)
                    .build();
            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose.mirror(),farParkPose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farParkPose.mirror().getHeading())
                    .setBrakingStart(2.5)
                    .build();
        }

        robot.follower.setMaxPower(1);
        Robot.sort = false;
        robot.limelight.setPipLine(Robot.color);


        while (!isStopRequested()){

           if (stage == 1 ){
                processor.override(new Shoot());
                increment();
            } else if (stage == 2 && !processor.isBusy()) {
                processor.override(new IntakeBall(true));
                robot.follower.followPath(grabPickup1);
                increment();
            } else if (stage == 3 && moveToNext()) {
               processor.override(new IntakeTransfer());
                robot.follower.followPath(scorePickup1);
                increment();
            } else if (stage == 4 && robot.follower.atParametricEnd()) {
                processor.override(new Shoot());
                increment();
            } else if (stage == 5 && !processor.isBusy()) {
                processor.override(new IntakeBall(false));
                robot.follower.followPath(grabPickup2);
                increment();
            } else if (stage == 6 && moveToNext()) {
               if (robot.colorSensors.ballsInIntake()){
                   processor.override(new IntakeTransfer());
               }
                robot.follower.followPath(scorePickup2);
                increment();
            } else if (stage == 7 && robot.follower.atParametricEnd()) {
                processor.override(new Shoot());
                increment();
            } else if (stage == 8 && !processor.isBusy()) {

                processor.override(new IntakeBall(false));
                robot.follower.followPath(grabPickup3);
                increment();

            } else if (stage == 9 && moveToNext()) {
               if (robot.colorSensors.ballsInIntake()){
                   processor.override(new IntakeTransfer());
               }
               if (elapsedTime.milliseconds()>500){
                   increment();
               }
               robot.follower.followPath(scorePickup3);

            } else if (stage == 10 && robot.follower.atParametricEnd()) {
                processor.override(new Shoot());
                increment();
            } else if (stage == 11 && !processor.isBusy()) {
                processor.override(new IntakeBall(true));
                robot.follower.followPath(grabPickup4);
                increment();
            } else if (stage == 12 && moveToNext()) {
               processor.override(new IntakeTransfer());
                robot.follower.followPath(scorePickup4);
                increment();
            } else if (stage == 13 && robot.follower.atParametricEnd()) {
                processor.override(new Shoot());
                increment();
            }else if (stage == 14 && !processor.isBusy()) {
                processor.override(new IntakeBall(true));
                robot.follower.followPath(grabPickup5);
                increment();
            } else if (stage == 15 && moveToNext()) {
               processor.override(new IntakeTransfer());
                robot.follower.followPath(scorePickup5);
                increment();
            } else if (stage == 16 && robot.follower.atParametricEnd()) {
                processor.override(new Shoot());
                increment();
            } else if (stage == 17 && !processor.isBusy()) {
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

    private boolean moveToNext(){
        return  robot.follower.atParametricEnd() || Robot.threeBalls;
    }
}

