package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.command.Sort;
import org.firstinspires.ftc.teamcode.common.enums.Color;
import org.firstinspires.ftc.teamcode.common.enums.Position;
import org.firstinspires.ftc.teamcode.common.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lights;
import org.firstinspires.ftc.teamcode.common.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.common.subsystems.Pins;
import org.firstinspires.ftc.teamcode.common.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.common.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
public class Robot {

    private long lastTime;

    private Pose goalPos;
    private final Pose redGoalPos = new Pose(133, 131, Math.toRadians(45));
    private final Pose blueGoalPos = new Pose(11, 131, Math.toRadians(135));
    private double turretTargetAngle = 0;

    public static Pose robotPos;

    public static Color color ;

    public static int motif;

    public static Position position;

    public static Boolean useAutoAim;


    public static Boolean sort;

    public Follower follower;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;

    public Limelight limelight;

    public Pins pins;
    public ColorSensors colorSensors;
    private final TimestampedAngleBacklog backlog = new TimestampedAngleBacklog();
    public Lights lights;
    public Turret turret;


    public Robot(HardwareMap hardwareMap){

        double dt = System.currentTimeMillis() - lastTime;
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        pins = new Pins(hardwareMap);
        colorSensors = new ColorSensors(hardwareMap);
        lights = new Lights(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
        lastTime = System.currentTimeMillis();
    }

    public void update(){
        setGoalPose();
        robotPos = follower.getPose();
        backlog.addAngle(turret.getTurretAngle());
        limelight.update();
        follower.update();
        shooter.update(getShooterTargetVelocity(), getFlapPos(getShooterTargetVelocity()));
        turret.update(getTurretTarget());
        if (sort){
            lights.setLights(colorSensors.getColors());
        } else {
            lights.setToDefault();
        }






    }
    private double getShooterTargetVelocity(){
        double distance;
        if (limelight.getDistance() != 0){
            distance = limelight.getDistance();
        } else {
            if (color == Color.BLUE){
                distance = robotPos.distanceFrom(blueGoalPos);
            } else {
                distance = robotPos.distanceFrom(redGoalPos);
            }
        }
        double velocity =  (0.027305* Math.pow(distance, 2)) - (0.0836134* distance) + 925.12785;
        if (useAutoAim){
            if (velocity > 1600){
                return 1500;
            } else {
                return velocity;
            }
        } else {
            return 1200;
        }

    };
    private double getFlapPos(Double velocity){
        if (velocity < 1300){
            return 0.425;
        }else{
            return 0.425;
        }
    }

    private double getTurretTarget(){
        double targetAngle;

        if (limelight.limeLightResult()){
            double tx = limelight.getResult().getTx();
            double prevAngle = backlog.getAngle(limelight.getResult().getControlHubTimeStampNanos() );
            targetAngle = prevAngle - tx;
        } else {
            double gobleAngle = Math.atan2(goalPos.getY() - robotPos.getY(), goalPos.getX() - robotPos.getX());
            double turretHeading = robotPos.getHeading() + Math.PI;
            if (turretHeading > Math.PI) turretHeading -= 2 * Math.PI;
            if (turretHeading < -Math.PI) turretHeading += 2 * Math.PI;
            targetAngle = Math.toDegrees(gobleAngle - turretHeading);
            if (targetAngle > 180) targetAngle -= 360;
        }
        if (useAutoAim){
            return targetAngle;
        } else {
            return turretTargetAngle;
        }
    }
    public void setTurretAngle(double turretTargetAngle){
        this.turretTargetAngle = turretTargetAngle;
    }
    private void setGoalPose(){
        if (Robot.color == Color.RED){
            goalPos = redGoalPos;
        } else {
            goalPos = blueGoalPos;
        }
    }
}
