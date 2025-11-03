package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {

    private long lastTime;
    public Follower follower;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;


    public Robot(HardwareMap hardwareMap){

        double dt = System.currentTimeMillis() - lastTime;
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        lastTime = System.currentTimeMillis();
    }

    public void update(){
        follower.update();
        shooter.update();

    }
}
