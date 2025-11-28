package org.firstinspires.ftc.teamcode.common.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

public class ReverseShooter implements Command{
    int stage = 0;

    @Override
    public void update(Robot robot) {
        if (stage == 0){
            robot.shooter.reverseShooter();


            stage++;
        }
    }

    @Override
    public boolean isDone() {
        return stage == 1;
    }
}
