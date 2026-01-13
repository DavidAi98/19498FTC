package org.firstinspires.ftc.teamcode.TeleOp;

public class Artifact {
    private String color;
    private double position;
    public Artifact(String color, double position){
        this.color = color;
        this.position= position;
    }
    public String getColor(){
        return color;
    }
    public double getPosition(){
        return position;
    }

}
