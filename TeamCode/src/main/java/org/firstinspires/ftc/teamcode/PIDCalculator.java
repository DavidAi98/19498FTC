package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDCalculator{
    public double kP, kI, kD;
    ElapsedTime timer = new ElapsedTime();
    public double lastTime,lastError, integral;
    public PIDCalculator(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void reset(){
        integral = 0;
        lastTime = timer.seconds();
        lastError = 0;
    }

    public void reset(double time, double error){
        lastTime = time;
        lastError = error;
    }
    public double calculator(double target, double current){
        double error = target - current;
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        double derivative = 0;
        if (dt > 0){
            integral += error * dt;
            derivative = (error - lastError)/dt;
        }
        double result = kP * error + kI * integral + kD * derivative;
        reset(currentTime, error);
        return result;
    }


}