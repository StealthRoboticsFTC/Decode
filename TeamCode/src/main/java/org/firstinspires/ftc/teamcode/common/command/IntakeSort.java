package org.firstinspires.ftc.teamcode.common.command;

import org.firstinspires.ftc.teamcode.common.Robot;

public class IntakeSort implements Command{
    private int stage = 0;
    @Override
    public void update(Robot robot) {
        if (stage == 0){
            robot.intake.turnOnIntake();
            robot.pins.closeAllPin();
            robot.lifter.liftBlock();
            stage++;
        }

    }

    @Override
    public boolean isDone() {
        return stage == 1;
    }
}
