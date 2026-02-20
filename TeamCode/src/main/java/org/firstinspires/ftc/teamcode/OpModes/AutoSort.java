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
import org.firstinspires.ftc.teamcode.common.command.Sort;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;
import org.firstinspires.ftc.teamcode.common.enums.Color;
import org.firstinspires.ftc.teamcode.common.menu.Menu;
import org.firstinspires.ftc.teamcode.common.menu.Screen;

import java.util.Arrays;
import java.util.List;


public class AutoSort extends LinearOpMode {
    Robot robot;
    Processor processor;
    ElapsedTime elapsedTime;
    private static int stage;
    private static final int shootTime = 5000;

    private final Pose redStartPose = new Pose(120, 129, Math.toRadians(315));
    private final Pose redScore1Pose = new Pose(90,99 ,Math.toRadians(300));
    private final Pose redScore2Pose = new Pose(90,99, Math.toRadians(300));
    private final Pose redPickup2Pose = new Pose(128,63, Math.toRadians(290));
    private final Pose redPickup1Pose = new Pose(123,93, Math.toRadians(285));

    private final Pose redControl = new Pose(103, 112);
    private final Pose redParkPose = new Pose(120,70, Math.toRadians(270));
    private final Pose blueStartPose = new Pose(120, 129, Math.toRadians(315)).mirror();
    private final Pose blueScore1Pose = new Pose(90,99 ,Math.toRadians(300)).mirror();
    private final Pose blueScore2Pose = new Pose(90,99, Math.toRadians(300)).mirror();
    private final Pose bluePickup2Pose = new Pose(130,63, Math.toRadians(290)).mirror();
    private final Pose bluePickup1Pose = new Pose(125,93, Math.toRadians(285)).mirror();

    private final Pose blueControl = new Pose(103, 112).mirror();

    private final Pose blueParkPose = new Pose(120,70, Math.toRadians(270)).mirror();



    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2,  park;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        processor = new Processor();
        elapsedTime = new ElapsedTime();
        stage = 1;


        Menu menu = new Menu(new ButtonListener(gamepad1), telemetry);
        menu.addScreen(new Screen("Color", Arrays.asList(Color.values())));


        menu.runBlocking();

        List<Object> options = menu.getSelections();
        Robot.color = (Color) options.get(0);
        robot.pins.closeAllPin();

        waitForStart();

        if (Robot.color == Color.BLUE){
            robot.follower.setStartingPose(blueStartPose);

            scorePreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(blueStartPose, blueScore1Pose))
                    .setLinearHeadingInterpolation(blueStartPose.getHeading(), blueScore1Pose.getHeading())
                    .build();


            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(blueScore1Pose, blueControl, bluePickup1Pose))
                    .setLinearHeadingInterpolation(blueScore1Pose.getHeading(), bluePickup1Pose.getHeading())

                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(bluePickup1Pose, blueScore2Pose))
                    .setLinearHeadingInterpolation(bluePickup1Pose.getHeading(), blueScore2Pose.getHeading())
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(blueScore2Pose, bluePickup2Pose))
                    .setLinearHeadingInterpolation(blueScore2Pose.getHeading(), bluePickup2Pose.getHeading())
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(bluePickup2Pose, blueScore2Pose))
                    .setLinearHeadingInterpolation(bluePickup2Pose.getHeading(), blueScore2Pose.getHeading())
                    .build();

            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(blueScore2Pose, blueParkPose))
                    .setLinearHeadingInterpolation(blueScore2Pose.getHeading(), blueParkPose.getHeading())
                    .build();
        }
        else if (Robot.color == Color.RED ){
            robot.follower.setStartingPose(redStartPose);

            scorePreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(redStartPose, redScore1Pose))
                    .setLinearHeadingInterpolation(redStartPose.getHeading(), redScore1Pose.getHeading())
                    .build();


            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(redScore1Pose, redControl, redPickup1Pose))
                    .setLinearHeadingInterpolation(redScore1Pose.getHeading(), redPickup1Pose.getHeading())

                    .build();
            scorePickup1= robot.follower.pathBuilder()
                    .addPath(new BezierLine(redPickup1Pose, redScore2Pose))
                    .setLinearHeadingInterpolation(redPickup1Pose.getHeading(), redScore2Pose.getHeading())
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(redScore2Pose, redPickup2Pose))
                    .setLinearHeadingInterpolation(redScore2Pose.getHeading(), redPickup2Pose.getHeading())
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(redPickup2Pose, redScore2Pose))
                    .setLinearHeadingInterpolation(redPickup2Pose.getHeading(), redScore2Pose.getHeading())
                    .build();

            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(redScore2Pose, redParkPose))
                    .setLinearHeadingInterpolation(redScore1Pose.getHeading(), redParkPose.getHeading())
                    .build();
        }

        robot.follower.setMaxPower(1);
        Robot.sort = true;

        robot.limelight.setPipLine(null);
        while (!isStopRequested()){
            if (stage == 1){
                processor.override(new IntakeBall(true));
                if (Robot.color == Color.RED){
                    robot.setTurretAngle(-10);
                } else {
                    robot.setTurretAngle(25);
                }
                increment();
            } else if (stage == 2) {
                robot.follower.followPath(scorePreload);
                increment();
            } else if (stage == 3 && !robot.follower.isBusy()) {
                Robot.motif = robot.limelight.getMotif();
                if (Robot.motif == 21 || Robot.motif == 22 || Robot.motif == 23){
                    robot.limelight.setPipLine(Robot.color);
                    if (Robot.color == Color.RED){
                        robot.setTurretAngle(-105);
                    } else {
                        robot.setTurretAngle(105);
                    }
                    increment();
                }


            } else if (stage == 4 ) {

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
            telemetry.addData("turretPos", robot.turret.getTurretAngle());
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
