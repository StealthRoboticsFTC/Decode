package org.firstinspires.ftc.teamcode.common.command;

import org.firstinspires.ftc.teamcode.common.Robot;

public class IntakeBall implements Command{
    private int stage = 0;
    boolean allPinsDown;
    public IntakeBall(Boolean allPinsDown){
        this.allPinsDown = allPinsDown;
    }
    @Override
    public void update(Robot robot) {
        if (stage == 0 ){

            robot.transfer.reverseTransfer();
            robot.pins.setPinsToIntake();
            robot.intake.turnOnIntake();

            stage++;
        } else if (stage == 1) {
            if (allPinsDown){
                robot.pins.closeAllPin();
                stage++;
            } else {
                robot.pins.setPinsToIntake();
                stage++;
            }
        }

    }

    @Override
    public boolean isDone() {
        return stage==2;
    }
}

