package org.firstinspires.ftc.teamcode.common.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

public class Shoot implements Command{
    private int stage = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    @Override
    public void update(Robot robot) {
        if (stage == 0){

            robot.intake.turnOnIntake();
            robot.transfer.turnOnTransfer();
            robot.pins.setPinOpen(1);
            elapsedTime.reset();
            stage++;
        } else if (stage == 1 && elapsedTime.milliseconds()>250) {
            robot.pins.setPinOpen(0);
            elapsedTime.reset();
            stage++;
        } else if (stage == 2 && elapsedTime.milliseconds() > 250) {
            robot.pins.openAllPin();
            stage++;
        }

    }

    @Override
    public boolean isDone() {
        return stage==3;
    }
}

