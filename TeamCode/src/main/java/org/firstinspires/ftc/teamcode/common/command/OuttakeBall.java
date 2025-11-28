package org.firstinspires.ftc.teamcode.common.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

public class OuttakeBall implements Command{
    private int stage = 0;

    ElapsedTime elapsedTime = new ElapsedTime();


    @Override
    public void update(Robot robot) {


        if (stage == 0){
            robot.shooter.shooterOff();
            robot.transfer.turnOffTransfer();
            robot.intake.outtake();
            elapsedTime.reset();

            stage++;
       } else if (stage == 1 && elapsedTime.milliseconds()>2000) {
            robot.intake.turnOnIntake();
            stage++;
        }
    }

    @Override
    public boolean isDone() {
        return stage==2;
    }
}

