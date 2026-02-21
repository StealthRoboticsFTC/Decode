package org.firstinspires.ftc.teamcode.common.command;

import org.firstinspires.ftc.teamcode.common.Robot;

public class Reset implements Command{
    int stage = 0;
    @Override
    public void update(Robot robot) {
        if (stage == 0){
            robot.shooter.setTargetVelocity(0);
            robot.pins.openAllPin();
            robot.turret.setTargetAngle(0);
            robot.intake.turnOffIntake();
            robot.lifter.liftDown();
            robot.transfer.turnOffTransfer();
        }
    }

    @Override
    public boolean isDone() {
        return stage == 1;
    }
}
