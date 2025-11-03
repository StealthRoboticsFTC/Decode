package org.firstinspires.ftc.teamcode.common.command;

import org.firstinspires.ftc.teamcode.common.Robot;

public class ShootClose implements Command{
    @Override
    public void update(Robot robot) {
        robot.intake.turnOnIntake();
        robot.transfer.turnOnTransfer();
        robot.shooter.shootClose();

    }

    @Override
    public boolean isDone() {
        return false;
    }
}
