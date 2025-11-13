package org.firstinspires.ftc.teamcode.common.command;

import org.firstinspires.ftc.teamcode.common.Robot;

public class ShootBlueAuto implements Command{
    int stage = 0;
    @Override
    public void update(Robot robot) {
        if (stage == 0){
            robot.turret.moveTurretBlueAuto();

            robot.intake.turnOffIntake();

            robot.shooter.shootClose();
            stage++;
        } else if (stage == 1 && robot.shooter.atTargetVelocity()) {

            robot.intake.turnOnIntake();
            robot.transfer.turnOnTransfer();
            stage++;
        }
    }

    @Override
    public boolean isDone() {
        return stage == 2;
    }
}
