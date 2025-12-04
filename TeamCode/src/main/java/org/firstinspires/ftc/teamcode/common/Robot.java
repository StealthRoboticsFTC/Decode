package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.enums.Color;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.common.subsystems.Pins;
import org.firstinspires.ftc.teamcode.common.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.common.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
public class Robot {

    private long lastTime;

    public static Color color ;

    public static Boolean useAutoAim;
    public static Boolean manul;
    public Follower follower;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;

    public Limelight limelight;

    public Pins pins;
    public Turret turret;


    public Robot(HardwareMap hardwareMap){

        double dt = System.currentTimeMillis() - lastTime;
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        pins = new Pins(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
        lastTime = System.currentTimeMillis();
    }

    public void update(){
        limelight.update();
        follower.update();
        shooter.update(limelight.getShooterTargetVelocity(), limelight.getFlapPos());
        turret.update(limelight.turretCurrentPos());


    }
}
