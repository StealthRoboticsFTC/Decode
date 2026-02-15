package org.firstinspires.ftc.teamcode.common;

public class CustomPIDController {
    private double kP, kI, kD, kF, kS;

    private double target, errorIntegral;
    private long lastUpdateTime;
    public CustomPIDController() {}

    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    public void setkS(double kS) {
        this.kS = kS;
    }

    public void setTarget(double pos) {
        target = pos;
    }

    public double getTarget(){return target;}

    public double update(double pos, double vel) {
        long time = System.nanoTime();
        double posError = target - pos;
        double targetVel = kP * posError;
        double velError = targetVel - vel;
        if (lastUpdateTime != 0) {
            double elapsedTime = (time - lastUpdateTime) * 1.0e-9;
            errorIntegral += kI * velError * elapsedTime;
            errorIntegral = Math.clamp(errorIntegral, -0.4, 0.4);
        }
        lastUpdateTime = time;
        double out = kF * targetVel + kD * velError + errorIntegral;
        return out + kS * Math.signum(posError);
    }

    public void skip() {
        lastUpdateTime = System.nanoTime();
        errorIntegral = 0;
    }
}
