package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.enums.Color;

import java.util.List;

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
        } else  {
            limelight.pipelineSwitch(1);

        }
    }

    public double getDistance(){
        if (result != null && result.isValid() ){
            double x = result.getTy();
            return (-0.0000316774*Math.pow(x, 3))+(0.0125026*Math.pow(x, 2))+(-1.69672*x) + 65.88894;

        } else {
            return 0;
        }

    }

    public int getMotif(){
        limelight.pipelineSwitch(2);
        int id = 0;
        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult : fiducialResults){
                id = fiducialResult.getFiducialId();
            }
           
        }
        return id;

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
