package org.firstinspires.ftc.teamcode.common.command;

import org.firstinspires.ftc.teamcode.common.Robot;

public class Reset implements Command{
    int stage = 0;
    @Override
    public void update(Robot robot) {
        if (stage == 0){
            robot.shooter.shooterOff();
            robot.transfer.turnOffTransfer();
            robot.intake.turnOffIntake();
            robot.turret.turretReset();

            stage++;
        }
    }

    @Override
    public boolean isDone() {
        return stage == 1;
    }
}
