package org.firstinspires.ftc.teamcode.common.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.enums.BallColors;

import java.util.ArrayList;
import java.util.List;

public class Sort implements Command{
    int stage = 0;
    BallColors[] motif;
    List<BallColors> currentColors= new ArrayList<>();

    ElapsedTime time = new ElapsedTime();
    public Sort(int id){
        if (id == 21){
            motif = new BallColors[]{BallColors.Green, BallColors.Purple, BallColors.Purple};
        } else if (id == 22) {
            motif = new BallColors[]{BallColors.Purple, BallColors.Green, BallColors.Purple};
        } else if (id == 23) {
            motif = new BallColors[]{BallColors.Purple, BallColors.Purple, BallColors.Green};
        }
    }
    @Override
    public void update(Robot robot) {
        if (stage == 0){
            robot.intake.turnOnIntake();
            robot.transfer.turnOnTransfer();
            robot.lifter.liftDown();
            currentColors = robot.colorSensors.getColors();
            if (!currentColors.contains(BallColors.None) || time.milliseconds()>1000){
                stage++;
            } 
        } else if (stage == 1){
            int index = currentColors.indexOf(motif[0]);
            if (index < 0){
                stage = 4;
            }else {
                robot.pins.setPinOpen(index);
                currentColors.set(index, BallColors.None);
                time.reset();
                stage++;
            }
        } else if (stage== 2 && time.milliseconds()>1000) {
            int index = currentColors.indexOf(motif[1]);
            if (index < 0){
                stage = 4;
            }else {
                robot.pins.setPinOpen(index);
                currentColors.set(index, BallColors.None);
                time.reset();
                stage++;
            }
        } else if (stage== 3 && time.milliseconds()>1000) {
            int index = currentColors.indexOf(motif[2]);
            if (index < 0){
                stage = 4;
            }else {

                robot.pins.setPinOpen(index);
                currentColors.set(index, BallColors.None);
                time.reset();
                stage++;
            }
        } else if (stage == 4) {
            robot.pins.openAllPin();
            stage++;
        }
    }



    @Override
    public boolean isDone() {
        return stage == 5;
    }
}
