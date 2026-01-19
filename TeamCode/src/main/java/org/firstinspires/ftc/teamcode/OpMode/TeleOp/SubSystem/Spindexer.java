package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class Spindexer {
    public DcMotor intake;
    public Servo spindexer1, spindexer2, leftPivot, rightPivot;
    public ColorSensor colorSensor;

    public ArrayList<Artifact> artifacts = new ArrayList<>();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime pivotTimer = new ElapsedTime();

    public int intakeStage = -1;
    public int outtakeStage = -1;
    private int intakeIndex = 1;
    private double nearestPosition = 0;
    private int nearestIndex = -1;

    public Spindexer(@NonNull HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "IntakeMotor");
        spindexer1 = hwMap.get(Servo.class, "spindexer1");
        spindexer2 = hwMap.get(Servo.class, "spindexer2");
        leftPivot = hwMap.get(Servo.class, "LeftPivot");
        rightPivot = hwMap.get(Servo.class, "RightPivot");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        rightPivot.setDirection(Servo.Direction.REVERSE);
        setSpindexer(Constant.SPINDEXER_INTAKE_POS1);
        setPivot(Constant.PIVOT_DOWN);
        intake.setPower(0);
    }

    // --- High Level Commands called by TeleOp ---

    public void startIntake() { intakeStage = 0; }
    public void stopIntake() { intakeStage = -1; intake.setPower(0); }
    public void startOuttake() { outtakeStage = 1; }
    public void stopOuttake() { outtakeStage = -1; }

    // --- The Logic Loop (Called once per TeleOp loop) ---

    public void update(boolean fireButton, boolean shooterReady) {
        handleIntakeLogic();
        handleOuttakeLogic(fireButton, shooterReady);
    }

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
        public String toString() { return color + " " + position;}
    }

    private void handleIntakeLogic() {
        intake.setPower(intakeStage != -1 ? Constant.INTAKE_POWER : 0);

        switch (intakeStage) {
            case 0: // Detection
                if (artifacts.size() < 3 && colorSensor.blue() >= 100 && colorSensor.green() >= 100) {
                    String color = (colorSensor.blue() > colorSensor.green()) ? "P" : "G";
                    artifacts.add(new Artifact(color, getOuttakePos(intakeIndex)));
                    intakeStage = 1;
                }
                break;
            case 1: // Indexing
                intakeIndex = (intakeIndex % 3) + 1;
                double pos = getIntakePos(intakeIndex);
                setSpindexer(pos);
                stateTimer.reset();
                intakeStage = 2;
                break;
            case 2: // Wait for mechanical movement
                if (artifacts.size() < 3){
                    if (stateTimer.milliseconds() >= 300) // SWAP OUT WITH ENCODER
                        intakeStage = 0;
                }
                else
                    intakeStage = -1;
                break;
        }
    }

    private void handleOuttakeLogic(boolean fireButton, boolean shooterReady) {
        if (outtakeStage == -1) return;

        switch (outtakeStage) {
            case 1: // Find Nearest / Align
                if (nearestIndex == -1) {
                    nearestIndex = (artifacts.size() == 3) ? 3 : 1;
                    nearestPosition = getOuttakePos(nearestIndex);
                }
                setSpindexer(nearestPosition);
                stateTimer.reset();
                outtakeStage = 2;
                break;
            case 2: // Fire Readiness
                if (stateTimer.milliseconds() >= 200 && shooterReady && fireButton) { // SWAP OUT WITH ENCODER
                    artifacts.remove(nearestIndex - 1);
                    setPivot(Constant.PIVOT_UP);
                    pivotTimer.reset();
                    outtakeStage = 3;
                }
                break;
            case 3: // Reset Pivot
                if (pivotTimer.milliseconds() >= 200) { // SWAP OUT WITH ENCODER
                    setPivot(Constant.PIVOT_DOWN);
                    outtakeStage = artifacts.isEmpty() ? -1 : 1;
                    nearestIndex = -1;
                }
                break;
        }
    }

    // Helper methods

    public void setSpindexer(double p) { spindexer1.setPosition(p+Constant.SPINDEXER_ANTIBACKLASH); spindexer2.setPosition(p-Constant.SPINDEXER_ANTIBACKLASH); }
    public void setPivot(double p) { leftPivot.setPosition(p); rightPivot.setPosition(p); }
    public double getIntakePos(int i) { return (i==1)?Constant.SPINDEXER_INTAKE_POS1 : (i==2)?Constant.SPINDEXER_INTAKE_POS2 : Constant.SPINDEXER_INTAKE_POS3; }
    public double getOuttakePos(int i) { return (i==1)?Constant.SPINDEXER_OUTTAKE_POS1 : (i==2)?Constant.SPINDEXER_OUTTAKE_POS2 : Constant.SPINDEXER_OUTTAKE_POS3; }
    public int getIntakeIndex(){
        return intakeIndex;
    }
}