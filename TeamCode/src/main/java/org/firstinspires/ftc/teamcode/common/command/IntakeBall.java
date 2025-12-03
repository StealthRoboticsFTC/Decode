package org.firstinspires.ftc.teamcode.common.command;

import org.firstinspires.ftc.teamcode.common.Robot;

public class IntakeBall implements Command{
    private int stage = 0;
    @Override
    public void update(Robot robot) {
        if (stage == 0){

            robot.transfer.reverseTransfer();
            robot.shooter.shooterOff();
            robot.intake.turnOnIntake();
            Robot.useAutoAim = false;

            stage++;
        }

    }

    @Override
    public boolean isDone() {
        return stage==1;
    }
}

