package org.firstinspires.ftc.teamcode.common.subsystems;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.Robot;
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
        } else if (color == Color.BLUE) {
            limelight.pipelineSwitch(1);
        } else {
            limelight.pipelineSwitch(2);
        }


        }

    public double getDistance(){
        if (result != null && result.isValid() ){
            double x = result.getTy();
            return  (0.00120297 * Math.pow(x, 4)) - (0.0134761 * Math.pow(x, 3)) + (0.0263289 *Math.pow(x, 2)) - (2.55705 * x) + 73.78656;

        } else {
            return 0;
        }

    }

    public int getMotif(){
        int id = 0;
        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult : fiducialResults){
                id = fiducialResult.getFiducialId();
            }
           
        }
        return id;

    }


    public LLResult getResult(){

        return result;

    }


    public boolean limeLightResult(){
        return result != null && result.isValid();
        }



    public void update(){
        limelight.updateRobotOrientation(Math.toDegrees(Robot.robotPos.getHeading()));
        result = limelight.getLatestResult();
    }
}
