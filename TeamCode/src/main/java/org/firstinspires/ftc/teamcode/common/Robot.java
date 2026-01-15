package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private final Pose redGoalPos = new Pose(144, 144, Math.toRadians(45));
    private final Pose blueGoalPos = new Pose(0, 144, Math.toRadians(135));

    public static Pose robotPos;

    public static Color color ;

    public static Position position;

    public static Boolean useAutoAim;
    public static Boolean manul;

    public static Boolean sort;
    public Follower follower;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;

    public Limelight limelight;

    public Pins pins;
    public ColorSensors colorSensors;
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
        limelight.update();
        follower.update();
        robotPos = follower.getPose();
        shooter.update(getShooterTargetVelocity(), getFlapPos(getShooterTargetVelocity()));
        turret.update(limelight.turretCurrentPos());
        lights.setLights(colorSensors.getColors());


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
        double velocity =  (0.0201257 * Math.pow(distance, 2)) + (1.08993 * distance) + +880.67132;
        if (velocity > 1600){
            return 1500;
        } else {
            return velocity;
        }

    };
    private double getFlapPos(Double velocity){
        if (velocity < 1300){
            return 0.55;
        }else{
            return 0.45;
        }
    }
}
