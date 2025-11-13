package org.firstinspires.ftc.teamcode.common.command;

import org.firstinspires.ftc.teamcode.common.Robot;

public class ShootMedium implements Command{
    private int stage = 0;
    @Override
    public void update(Robot robot) {
        if (stage == 0){

            robot.intake.turnOffIntake();
            robot.shooter.shootMedium();
            stage++;
        } else if (stage == 1 && robot.shooter.atTargetVelocity()) {
            robot.intake.turnOnIntake();
            robot.transfer.turnOnTransfer();
            stage++;
        }

    }

    @Override
    public boolean isDone() {
        return stage==2;
    }
}

