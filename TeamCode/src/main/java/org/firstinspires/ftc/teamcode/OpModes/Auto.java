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

    private final double heading = Math.toRadians(180);
    private final Pose farStart = new Pose(38,8, heading);
    private final Pose closeStart = new Pose(24, 129, Math.toRadians(225));

    private final Pose farScore1Pose = new Pose(40,8,heading);
    private final Pose farScore2Pose = new Pose(56,25, heading);
    private final Pose closeScore1Pose = new Pose(50,84,heading);
    private final Pose closeScore2Pose = new Pose(62,70, heading);
    private final Pose closeScore3Pose = new Pose(45,10,heading);


    private final Pose farPickup1Pose = new Pose(16,8, heading);
    private final Pose farPickup2Pose = new Pose(14,36, heading);
    private final Pose farPickup2Control = new Pose(45,35, heading);
    private final Pose farPickup3Pose = new Pose(15,66, heading);
    private final Pose farPickup3Control = new Pose(60,60, heading);
    private final Pose farPickup4Pose = new Pose(12, 18, heading);

    private final Pose farPickup5Pose = new Pose(10,36, heading);
    private final Pose closePickup1Pose = new Pose(15,84, heading);
    private final Pose closePickup2Pose = new Pose(13,58, heading);
    private final Pose closePickup2Control = new Pose(40, 58, heading);
    private final Pose closePickup3Pose = new Pose(13,36, heading);
    private final Pose closePickup3Control = new Pose(35,32, heading);
    private final Pose closePickup4Pose = new Pose(12, 10, heading);

    private final Pose closeGateOpen = new Pose(16, 65, heading);
    private final Pose closeGateOpenControl = new Pose(35, 65, heading);


    private final Pose farParkPose = new Pose(50,25, heading);
    private final Pose closeParkPose = new Pose(35, 10, heading);


    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, openGate, scorePickup2, grabPickup3, scorePickup3, grabPickup4, scorePickup4, grabPickup5, scorePickup5, park;
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

        robot.lifter.liftDown();

        if (Robot.color == Color.BLUE && Robot.position == Position.Far){
            robot.pins.preload();
            robot.follower.setStartingPose(farStart);



            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farStart,farPickup1Pose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farPickup1Pose.getHeading())
                    .setNoDeceleration()

                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup1Pose.getHeading(), farScore2Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose, farPickup2Control,farPickup2Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup2Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup2Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup2Pose.getHeading(), farScore2Pose.getHeading())
                    .setBrakingStart(3)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose, farPickup3Control, farPickup3Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup3Pose.getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup3Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup3Pose.getHeading(),farScore2Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();

            grabPickup4  = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore2Pose, farPickup4Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup4Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup4Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup4Pose.getHeading(), farScore2Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();

            grabPickup5  = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore2Pose, farPickup5Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup5Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup5Pose, farScore1Pose))
                    .setLinearHeadingInterpolation(farPickup5Pose.getHeading(), farScore1Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();


            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose,farParkPose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farParkPose.getHeading())
                    .build();
        }
        else if (Robot.color == Color.RED && Robot.position == Position.Far){
            robot.pins.preload();
            robot.follower.setStartingPose(farStart.mirror());



            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farStart.mirror(),farPickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farPickup1Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup1Pose.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose.mirror(), farPickup2Control.mirror(),farPickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup2Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup2Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup2Pose.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose.mirror(), farPickup3Control.mirror(), farPickup3Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup3Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()

                    .addPath(new BezierLine(farPickup3Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup3Pose.mirror().getHeading(),farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();
            grabPickup4  = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore2Pose.mirror(), farPickup4Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup4Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup4Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup4Pose.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();
            grabPickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore2Pose.mirror(), farPickup5Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup5Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup5Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup5Pose.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();

            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore2Pose.mirror(),farParkPose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farParkPose.mirror().getHeading())
                    .setBrakingStart(3)
                    .build();
        } else if (Robot.color == Color.BLUE && Robot.position == Position.Close){
            robot.pins.closeAllPin();
            robot.follower.setStartingPose(closeStart);

            scorePreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeStart, closeScore1Pose))
                    .setLinearHeadingInterpolation(closeStart.getHeading(), closeScore1Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();



            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore1Pose,closePickup1Pose))
                    .setLinearHeadingInterpolation(closeScore1Pose.getHeading(), closePickup1Pose.getHeading())
                    .setNoDeceleration()

                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup1Pose, closeScore2Pose))
                    .setLinearHeadingInterpolation(closePickup1Pose.getHeading(), closeScore2Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();

            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closeScore2Pose, closePickup2Control,closePickup2Pose))
                    .setLinearHeadingInterpolation(closeScore2Pose.getHeading(), closePickup2Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            openGate = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closePickup2Pose, closeGateOpenControl, closeGateOpen))
                    .setLinearHeadingInterpolation(closePickup2Pose.getHeading(), closeGateOpen.getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeGateOpen, closeScore2Pose))
                    .setLinearHeadingInterpolation(closeGateOpen.getHeading(), closeScore2Pose.getHeading())
                    .setBrakingStart(3)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closeScore2Pose, closePickup3Control, closePickup3Pose))
                    .setLinearHeadingInterpolation(closeScore2Pose.getHeading(), closePickup3Pose.getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup3Pose, closeScore3Pose))
                    .setLinearHeadingInterpolation(closePickup3Pose.getHeading(), closeScore3Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();

            grabPickup4  = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore3Pose, closePickup4Pose))
                    .setLinearHeadingInterpolation(closeScore3Pose.getHeading(), closePickup4Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup4Pose, closeScore3Pose))
                    .setLinearHeadingInterpolation(farPickup4Pose.getHeading(), closeScore3Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();


            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore3Pose,closeParkPose))
                    .setLinearHeadingInterpolation(closeScore1Pose.getHeading(), closeParkPose.getHeading())
                    .build();
        }
        else if (Robot.color == Color.RED && Robot.position == Position.Close){
            robot.pins.closeAllPin();
            robot.follower.setStartingPose(closeStart.mirror());

            scorePreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeStart.mirror(), closeScore1Pose.mirror()))
                    .setLinearHeadingInterpolation(closeStart.mirror().getHeading(), closeScore1Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();



            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore1Pose.mirror(),closePickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(closeScore1Pose.mirror().getHeading(), closePickup1Pose.mirror().getHeading())
                    .setNoDeceleration()

                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup1Pose.mirror(), closeScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(closePickup1Pose.mirror().getHeading(), closeScore2Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();

            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closeScore2Pose.mirror(), closePickup2Control.mirror(),closePickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(closeScore2Pose.mirror().getHeading(), closePickup2Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            openGate = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closePickup2Pose.mirror(), closeGateOpenControl.mirror(), closeGateOpen.mirror()))
                    .setLinearHeadingInterpolation(closePickup2Pose.mirror().getHeading(), closeGateOpen.mirror().getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeGateOpen.mirror(), closeScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(closeGateOpen.mirror().getHeading(), closeScore2Pose.mirror().getHeading())
                    .setBrakingStart(3)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closeScore2Pose.mirror(), closePickup3Control.mirror(), closePickup3Pose.mirror()))
                    .setLinearHeadingInterpolation(closeScore2Pose.mirror().getHeading(), closePickup3Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup3Pose.mirror(), closeScore3Pose.mirror()))
                    .setLinearHeadingInterpolation(closePickup3Pose.mirror().getHeading(), closeScore3Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();

            grabPickup4  = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore3Pose.mirror(), closePickup4Pose.mirror()))
                    .setLinearHeadingInterpolation(closeScore3Pose.mirror().getHeading(), closePickup4Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup4Pose.mirror(), closeScore3Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup4Pose.mirror().getHeading(), closeScore3Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();


            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore3Pose.mirror(),closeParkPose.mirror()))
                    .setLinearHeadingInterpolation(closeScore1Pose.mirror().getHeading(), closeParkPose.mirror().getHeading())
                    .build();
        }

        waitForStart();



        robot.follower.setMaxPower(1);
        Robot.sort = false;
        robot.limelight.setPipLine(Robot.color);


        while (!isStopRequested()){
            if (Robot.position == Position.Far){

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
                } else if (stage == 4 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 5 && !processor.isBusy()) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(grabPickup2);
                    increment();
                } else if (stage == 6 && moveToNext()) {

                    robot.follower.followPath(scorePickup2);
                    elapsedTime.reset();
                    stage = 100;
                } else if (stage == 100) {
                   if (robot.colorSensors.ballsInIntake()){
                       processor.override(new IntakeTransfer());
                       elapsedTime.reset();
                       stage = 7;
                   } else if (elapsedTime.milliseconds() > 1000){
                       elapsedTime.reset();
                       stage = 7;
                   }

               } else if (stage == 7 && !robot.follower.isBusy() && elapsedTime.milliseconds()>1500) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 8 && !processor.isBusy()) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(grabPickup3, 0.8, true);

                    increment();

                } else if (stage == 9 && moveToNext()) {
                   robot.follower.followPath(scorePickup3);
                   increment();

                } else if (stage == 10 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 11 && !processor.isBusy()) {
                    processor.override(new IntakeBall(true));
                    robot.follower.followPath(grabPickup4);
                    increment();
                } else if (stage == 12 && moveToNext() ) {
                   processor.override(new IntakeTransfer());
                    robot.follower.followPath(scorePickup4);
                    increment();
                } else if (stage == 13 && !robot.follower.isBusy()) {
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
                } else if (stage == 16 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 17 && !processor.isBusy()) {
                    robot.follower.followPath(park);
                    increment();

                }

           } else if (Robot.position == Position.Close) {
               if (stage == 1) {
                    processor.override(new IntakeBall(true));
                    robot.follower.followPath(scorePreload);
                    increment();
                } else if (stage == 2 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 3 && !processor.isBusy()) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(grabPickup1);
                    increment();
                } else if (stage == 4 && moveToNext()) {
                    robot.follower.followPath(scorePickup1);
                    elapsedTime.reset();
                    increment();
                } else if (stage == 5){
                    if (robot.colorSensors.ballsInIntake()){
                        processor.override(new IntakeTransfer());
                        increment();
                    } else if (elapsedTime.milliseconds() > 1000){
                        increment();
                    }

                } else if (stage == 6 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 8 && !processor.isBusy()) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(grabPickup2);
                    increment();

                } else if (stage == 9 && moveToNext()) {
                    robot.follower.followPath(openGate, 0.75, true);
                    increment();

                } else if (stage == 10 && moveToNext()) {
                   robot.follower.followPath(scorePickup2);
                   increment();
                } else if (stage == 11) {
                   if (robot.colorSensors.ballsInIntake()){
                       processor.override(new IntakeTransfer());
                       increment();
                   } else if (elapsedTime.milliseconds() > 1000){
                       increment();
                   }
                } else if (stage == 12 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                }else if (stage == 14 && !processor.isBusy()) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(grabPickup3);
                    increment();
                } else if (stage == 15 && moveToNext()) {
                    robot.follower.followPath(scorePickup3);
                    increment();
                } else if (stage == 16) {
                   if (robot.colorSensors.ballsInIntake()){
                       processor.override(new IntakeTransfer());
                       increment();
                   } else if (elapsedTime.milliseconds() > 1000){
                       increment();
                   }
                } else if (stage == 17 && !robot.follower.isBusy()){
                   processor.override(new Shoot());
                   increment();
               }else if (stage == 18 && !processor.isBusy()) {
                   processor.override(new IntakeBall(true));
                    robot.follower.followPath(grabPickup4);
                    increment();

                }else if (stage == 19 && moveToNext()) {
                    processor.override(new IntakeTransfer());
                    robot.follower.followPath(scorePickup4);
                    increment();

                } else if (stage == 20 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 21 && !processor.isBusy()) {
                    robot.follower.followPath(park);
                    increment();

                }
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

