package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.enums.Color;

public class Limelight {
    private final Limelight3A limelight;
    private LLResult result;




    public Limelight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limeLight");
        limelight.start();
        result = limelight.getLatestResult();

    }

    public void setPipLine(Color color){
        if (color == Color.RED){
            limelight.pipelineSwitch(0);
        } else if (color == Color.BLUE) {
            limelight.pipelineSwitch(1);

        } else{
            limelight.pipelineSwitch(2);
        }
    }

    public double getShooterTargetVelocity(){
        if (result != null && result.isValid() ){
            double x = result.getTy();
            return (0.00422705*Math.pow(x, 4))-(0.0604476*Math.pow(x, 3))+(0.0219134*Math.pow(x, 2))-(10.40972*x) + 1096.17776;

        } else {
            return 1000;
        }

    }

    public double getFlapPos(){
        if (result != null && result.isValid() ){
            double x = result.getTy();
            return 0.00211652 * x+0.666693;
        } else {
            return 0.7;
        }


    }

    public double turretCurrentPos(){
        if (result != null && result.isValid() ){
            return result.getTx();
        }
        else {
            return 0;
        }

    }

    public void update(){
        result = limelight.getLatestResult();
    }
}
