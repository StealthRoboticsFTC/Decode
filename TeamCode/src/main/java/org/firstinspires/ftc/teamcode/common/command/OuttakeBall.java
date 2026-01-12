package org.firstinspires.ftc.teamcode.common.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

public class OuttakeBall implements Command{
    private int stage = 0;




    @Override
    public void update(Robot robot) {


        if (stage == 0){

            robot.transfer.reverseTransfer();
            robot.intake.outtake();
            robot.pins.openAllPin();

            stage++;
       }
    }

    @Override
    public boolean isDone() {
        return stage==1;
    }
}

