package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.teamcode.OpMode.TeleOp.SubSystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class Spindexer {
    public DcMotorEx spindexerEncoder;
    public DcMotor intake;
    public Servo spindexer1, spindexer2, leftPivot, rightPivot;
    public ColorSensor colorSensor;

    public ArrayList<Artifact> artifacts = new ArrayList<>();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime pivotTimer = new ElapsedTime();

    public int intakeStage = -1;
    public int outtakeStage = -1;
    private int intakeIndex = 1;
    public double nearestPosition = 0;
    public int nearestIndex = -1;
    private boolean encoderResetDone = false;
    public int targetTicks;
    public int currentTicks;

    public Spindexer(@NonNull HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "IntakeMotor");
        spindexer1 = hwMap.get(Servo.class, "spindexer1");
        spindexer2 = hwMap.get(Servo.class, "spindexer2");
        leftPivot = hwMap.get(Servo.class, "LeftPivot");
        rightPivot = hwMap.get(Servo.class, "RightPivot");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        spindexerEncoder = hwMap.get(DcMotorEx.class, "RightFrontMotor");

        leftPivot.setDirection(Servo.Direction.REVERSE);
        setSpindexer(Constant.SPINDEXER_INTAKE_POS1);
        setPivot(Constant.PIVOT_DOWN);
        intake.setPower(0);
    }

    // --- High Level Commands called by TeleOp ---
    public void startIntake() { intakeStage = 0; }
    public void stopIntake() { intakeStage = -1; intake.setPower(0); }
    public void startOuttake() { outtakeStage = 1; }
    public void stopOuttake() { outtakeStage = -1; }

    public void update(boolean fireButton, boolean shooterReady) {
        currentTicks = spindexerEncoder.getCurrentPosition();
        handleIntakeLogic();
        handleOuttakeLogic(fireButton, shooterReady);
        if (!encoderResetDone) {
            spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderResetDone = true;
        }
    }

    public class Artifact {
        private String color;
        private double position;
        public Artifact(String color, double position){
            this.color = color;
            this.position= position;
        }
        public String getColor(){ return color; }
        public double getPosition(){ return position; }
        public String toString() { return color + " " + position;}
    }

    private void handleIntakeLogic() {
        intake.setPower(intakeStage != -1 ? Constant.INTAKE_POWER : 0);
        switch (intakeStage) {
            case 0:
                if (artifacts.size() < 3 && colorSensor.blue() >= 150 && colorSensor.green() >= 150) {
                    String color = (colorSensor.blue() >= colorSensor.green()) ? "P" : "G";
                    artifacts.add(new Artifact(color, getOuttakePos(intakeIndex)));
                    intakeStage = artifacts.size() < 3 ? 1 : -1;
                }
                break;
            case 1:
                intakeIndex = (intakeIndex % 3) + 1;
                setSpindexer(getIntakePos(intakeIndex));
                stateTimer.reset();
                intakeStage = 2;
                break;
            case 2:
                if (isSpindexerAtTarget(getSpindexTargetTicksIntake(intakeIndex))) {
                    intakeStage = (artifacts.size() < 3) ? 0 : -1;
                }
                break;
        }
    }

    private void handleOuttakeLogic(boolean fireButton, boolean shooterReady) {
        if (outtakeStage == -1 || (artifacts.isEmpty() && outtakeStage != 3)) {
            outtakeStage = -1;
            return;
        }

        switch (outtakeStage) {
            case 1: // Selection Logic: Sequence 2 -> 1 -> 3
                int count = artifacts.size();

                // If the ball at index 0 is at Position 2, it means we have Slot 2 and Slot 3 (but no Slot 1)
                // We check the first ball's position to see if it's Slot 2.
                boolean hasSlot2 = false;
                for(Artifact a : artifacts) if(a.getPosition() == getOuttakePos(2)) hasSlot2 = true;

                if (count == 3 || (count == 2 && hasSlot2)) {
                    // Target Slot 2 if it exists (Index 1 if 3 balls, Index 0 if 2 balls)
                    nearestIndex = 2;
                    nearestPosition = getOuttakePos(2);
                } else if (count == 2 || (count == 1 && artifacts.get(0).getPosition() == getOuttakePos(1))) {
                    nearestIndex = 1;
                    nearestPosition = getOuttakePos(1);
                } else {
                    nearestIndex = 3;
                    nearestPosition = getOuttakePos(3);
                }
                setSpindexer(nearestPosition);
                stateTimer.reset();
                outtakeStage = 2;
                break;
            case 2: // Fire Readiness
                targetTicks = getSpindexTargetTicksOuttake(nearestIndex);

                if (isSpindexerAtTarget(targetTicks) && shooterReady && fireButton) {
                    // Minimal removal logic: Find the ball that matches the physical slot we just fired
                    for (int i = 0; i < artifacts.size(); i++) {
                        if (artifacts.get(i).getPosition() == getOuttakePos(nearestIndex)) {
                            artifacts.remove(i);
                            break;
                        }
                    }
                    setPivot(Constant.PIVOT_UP);
                    pivotTimer.reset();
                    outtakeStage = 3;
                }
                break;
            case 3:
                if (pivotTimer.milliseconds() >= 125) {
                    setPivot(Constant.PIVOT_DOWN);
                    if (pivotTimer.milliseconds() >= (artifacts.isEmpty() ? 600 : 225)) {
                        if (artifacts.isEmpty()) {
                            outtakeStage = -1;
                            intakeIndex = 1; // RESET INTAKE TO SLOT 1 HERE
                        } else {
                            outtakeStage = 1;
                        }
                        nearestIndex = -1;
                    }
                }
                break;
        }
    }

    public void setSpindexer(double p) {
        spindexer1.setPosition(p + Constant.SPINDEXER_ANTIBACKLASH);
        spindexer2.setPosition(p - Constant.SPINDEXER_ANTIBACKLASH);
    }

    public void setPivot(double p) {
        leftPivot.setPosition(p + 0.01);
        rightPivot.setPosition(p);
    }

    public double getIntakePos(int i) {
        return (i==1) ? Constant.SPINDEXER_INTAKE_POS1 :
                (i==2) ? Constant.SPINDEXER_INTAKE_POS2 : Constant.SPINDEXER_INTAKE_POS3;
    }

    public double getOuttakePos(int i) {
        return (i==1) ? Constant.SPINDEXER_OUTTAKE_POS1 :
                (i==2) ? Constant.SPINDEXER_OUTTAKE_POS2 : Constant.SPINDEXER_OUTTAKE_POS3;
    }

    public int getIntakeIndex(){ return intakeIndex; }

    private boolean isSpindexerAtTarget(int targetTicks) {
        currentTicks = spindexerEncoder.getCurrentPosition();
        return Math.abs(currentTicks - targetTicks) <= Constant.SPINDEX_TICK_TOLERANCE;
    }

    private int getSpindexTargetTicksOuttake(int position) {
        switch (position) {
            case 1: return Constant.SPINDEXER_OUTTAKE_POS1_TICK;
            case 2: return Constant.SPINDEXER_OUTTAKE_POS2_TICK;
            case 3: return Constant.SPINDEXER_OUTTAKE_POS3_TICK;
            default: return Constant.SPINDEXER_OUTTAKE_POS1_TICK;
        }
    }
    private int getSpindexTargetTicksIntake(int position) {
        switch (position) {
            case 1: return Constant.SPINDEXER_INTAKE_POS1_TICK;
            case 2: return Constant.SPINDEXER_INTAKE_POS2_TICK;
            case 3: return Constant.SPINDEXER_INTAKE_POS3_TICK;
            default: return Constant.SPINDEXER_INTAKE_POS1_TICK;
        }
    }
}