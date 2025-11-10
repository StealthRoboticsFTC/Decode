package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Processor;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.IntakeBall;
import org.firstinspires.ftc.teamcode.common.command.ShootClose;

@Autonomous
public class BlueCloseAuto extends LinearOpMode {
    Robot robot;
    Processor processor;
    ElapsedTime elapsedTime;
    private static int stage;
    private final double heading = Math.toRadians(262.5);

    private final Pose startPose = new Pose(124,123,Math.toRadians(215)).mirror();
    private final Pose scorePose = new Pose(120,110, heading).mirror();
    private final Pose pickup1Pose = new Pose(120,80, heading).mirror();
    private final Pose pickup2Pose = new Pose(120,60, heading).mirror();
    private final Pose parkPose = new Pose(120,90, heading).mirror();



    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, park;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        processor = new Processor();
        elapsedTime = new ElapsedTime();
        stage = 0;

        robot.follower.setStartingPose(startPose);

        scorePreload = robot.follower.pathBuilder()

                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
        scorePickup1= robot.follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
        grabPickup2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
        scorePickup2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        park = robot.follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

        waitForStart();
        while (!isStopRequested()){
            if (stage == 0){
                robot.follower.setMaxPower(0.75);
                robot.follower.followPath(scorePreload);
                elapsedTime.reset();
                stage++;
            } else if (stage == 1 && elapsedTime.milliseconds() > 0) {
                processor.override(new ShootClose());
                elapsedTime.reset();
                stage++;

            } else if (stage == 2 && elapsedTime.milliseconds() > 5000) {
                processor.override(new IntakeBall());
                robot.follower.followPath(grabPickup1);
                elapsedTime.reset();
                stage++;

            } else if (stage == 3 && !robot.follower.isBusy()) {
                robot.follower.followPath(scorePickup1);
                elapsedTime.reset();
                stage++;

            } else if (stage == 4 && elapsedTime.milliseconds() > 0) {
                processor.override(new ShootClose());
                elapsedTime.reset();
                stage++;

            } else if (stage == 5 && elapsedTime.milliseconds() > 5000) {
                processor.override(new IntakeBall());
                robot.follower.followPath(grabPickup2);
                elapsedTime.reset();
                stage++;

            } else if (stage == 6 && !robot.follower.isBusy()) {
                robot.follower.followPath(scorePickup2);
                elapsedTime.reset();
                stage++;

            }else if (stage == 7 && elapsedTime.milliseconds() > 0) {
                processor.override(new ShootClose());
                elapsedTime.reset();
                stage++;

            } else if (stage == 8 && elapsedTime.milliseconds() > 5000) {
                processor.override(new IntakeBall());
                robot.intake.turnOffIntake();
                robot.follower.followPath(park);
                elapsedTime.reset();
                stage++;

            }
            robot.update();
            processor.update(robot);
        }

    }
}
