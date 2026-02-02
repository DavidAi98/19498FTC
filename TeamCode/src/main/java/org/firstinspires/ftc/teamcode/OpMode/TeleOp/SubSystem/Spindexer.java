package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Spindexer {
    public DcMotor spindexerEncoder, intake;
    public Servo spindexer1, spindexer2, leftPivot, rightPivot;
    public ColorSensor colorSensor1, colorSensor2;
    private VoltageSensor battery;

    public Artifact[] slots = new Artifact[3];
    public int artifactCount = 0;
    private String color;

    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime pivotTimer = new ElapsedTime();
    private ElapsedTime inverseTimer = new ElapsedTime();
    public ElapsedTime resetTimer = new ElapsedTime();
    private boolean intakeDone = false;
    public boolean encoderResetDone = false;

    public int intakeStage = -1, outtakeStage = -1;
    public int Index = 1, nearestIndex = -1;
    public int targetTicks, currentTicks;
    public double nearestPos, lastPos;
    private String targetColor = "NaN";

    private static final int[] priorityOrder = {2, 1, 3};
    private String motifLine = "";
    private int autonColor = 1;

    private String[][] shootMatrix = {
                {"acb", "bac", "cba"}, // GPP: line1->acb, line2->bac, line3->cba
                {"bac", "cba", "acb"}, // PGP: line1->bac, line2->cba, line3->acb
                {"cba", "acb", "bac"}  // PPG: line1->cba, line2->acb, line3->bac
        };


    public Spindexer(@NonNull HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "IntakeMotor");
        spindexer1 = hwMap.get(Servo.class, "spindexer1");
        spindexer2 = hwMap.get(Servo.class, "spindexer2");
        leftPivot = hwMap.get(Servo.class, "LeftPivot");
        rightPivot = hwMap.get(Servo.class, "RightPivot");
        colorSensor1 = hwMap.get(ColorSensor.class, "colorSensor1");
        colorSensor2 = hwMap.get(ColorSensor.class, "colorSensor2");
        spindexerEncoder = hwMap.get(DcMotor.class, "SpindexerEncoder");
        spindexerEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        battery = hwMap.voltageSensor.iterator().next();
        leftPivot.setDirection(Servo.Direction.REVERSE);

        setSpindexer(Constant.INTAKE_POS1);
        setPivot(Constant.PIVOT_DOWN);
        intake.setPower(0);
    }

    public void update(boolean fireButton, boolean shooterReady, boolean purpleButton, boolean greenButton, boolean skipSlotButton) {
        if (encoderResetDone) {
            handleIntakeLogic(skipSlotButton);
            handleOuttakeLogic(fireButton, shooterReady, purpleButton, greenButton);
        } else {
            intakeStage = -1;
            outtakeStage = -1;
            artifactCount = 0;
            Index = 1;
            setSpindexer(Constant.INTAKE_POS1);
            if (resetTimer.milliseconds() >= 2 * Constant.ANTI_STUCK_TIMER) {
                spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spindexerEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                encoderResetDone = true;
            }
        }
    }

    public void startIntake() {
        if (outtakeStage == -1 && artifactCount < 3) {
            setSpindexer(getIntakePos(Index));
            intakeStage = 1;
        }
    }
    public void stopIntake() { intake.setPower(0); intakeStage = -1; }

    public void inverseIntake() {
        if (!intakeDone) {
            intakeDone = true;
            inverseTimer.reset();
        }
        setSpindexer(Constant.OUTTAKE_POS2);
        if (inverseTimer.milliseconds() < Constant.INVERSE_TIMER && withinTarget(Constant.OUTTAKE_POS2_TICK, 400)) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }

    public void startOuttake() {
        if (intakeStage == -1 && artifactCount > 0) {
            outtakeStage = 1;
        }
    }
    public void stopOuttake() { outtakeStage = -1; }

    private void handleIntakeLogic(boolean skipSlot) {
        if (intakeStage == -1) {
            // Antistuck
            if (artifactCount == 3 && outtakeStage == -1) inverseIntake();
            else stopIntake();
            return;
        } else {
            intake.setPower(1);
        }

        switch (intakeStage) {
            case 1: // Color sensing
                boolean color1Detected = artifactCount < 3 && colorSensor1.blue() >= 150 && colorSensor1.green() >= 150;
                boolean color2Detected = artifactCount < 3 && colorSensor2.blue() >= 150 && colorSensor2.green() >= 150;
                if (skipSlot || color2Detected) {
                    color = (colorSensor2.blue() >= colorSensor2.green()) ? "P" : "G";

                    // Record artifact into slots Array
                    slots[Index - 1] = new Artifact(color, getOuttakePos(Index));
                    artifactCount++;

                    // Next stage if less than 3 artifacts
                    intakeStage = (artifactCount < 3) ? 2 : -1;
                }
                break;

            case 2: // Getting index for next slot
                Index++;
                if (Index > 3) Index = 1; // module
                // Move to target index
                setSpindexer(getIntakePos(Index));
                stateTimer.reset();
                intakeStage = 3;
                break;

            case 3: // Waiting for motion
                targetTicks = getIntakeTick(Index);
                boolean inSlot = withinTarget(targetTicks, Constant.INTAKE_TICK_TOLERANCE);
                if (inSlot) {
                    // Start detecting again
                    intakeStage = 1;
                } else if (stateTimer.milliseconds() > Constant.ANTI_STUCK_TIMER) {
                    resetTimer.reset();
                    encoderResetDone = false;
                }
                break;
        }
    }



    private void handleOuttakeLogic(boolean fireButton, boolean shooterReady, boolean purpleButton, boolean greenButton) {
        // Reduce runtime with return and setting IDLE state (-1)
        if (outtakeStage == -1 || (artifactCount == 0 && outtakeStage != 3)) {
            outtakeStage = -1;
            return;
        }

        switch (outtakeStage) {
            case 0: // Anti-stuck by going back to last outtake slot
                setSpindexer(lastPos);
                if (stateTimer.milliseconds() > 2 * Constant.ANTI_STUCK_TIMER) {
                    stateTimer.reset();
                    setSpindexer(nearestPos);
                    outtakeStage = 2;
                }
                break;

            case 1: // Selection Logic: 2 -> 1 -> 3
                int foundIndex = -1;
                // Trigger by Left Bumper or Dpad Up/Down
                if (fireButton)        targetColor = "ANY";
                else if (purpleButton) targetColor = "P";
                else if (greenButton)  targetColor = "G";

                // Fast leave to reduce runtime
                if (targetColor.equals("NaN")) return;

                for (int i : priorityOrder) {
                    Artifact a = slots[i - 1];
                    // Skip loop if target NOT FOUND
                    if (a == null) continue;
                    if (targetColor.equals("ANY") || a.getColor().equals(targetColor)) {
                        // Target index FOUND
                        foundIndex = i;
                        break;
                    }
                }

                // For antistuck
                lastPos = nearestPos;

                if (foundIndex != -1) {
                    nearestIndex = foundIndex;
                    nearestPos = getOuttakePos(nearestIndex);
                    setSpindexer(nearestPos);
                    stateTimer.reset();
                    outtakeStage = 2;
                } else if (targetColor.equals("G") || targetColor.equals("P")) {
                    // If can't find specific color, fire closet slot
                    targetColor = "ANY";
                }
                break;

            case 2: // Waiting for Fire
                targetTicks = getOuttakeTick(nearestIndex);
                boolean inSlot = withinTarget(targetTicks, Constant.OUTTAKE_TICK_TOLERANCE);

                // Check Spindex pos and shooter RPM
                if (inSlot && shooterReady) {
                    slots[nearestIndex - 1] = null;
                    artifactCount--;
                    setPivot(Constant.PIVOT_UP);
                    pivotTimer.reset();
                    outtakeStage = 3;
                    // Reset targetColor for next time
                    targetColor = "NaN";
                } else if (stateTimer.milliseconds() > Constant.ANTI_STUCK_TIMER && !inSlot) {
                    stateTimer.reset();
                    outtakeStage = 0;
                }
                break;

            case 3: // Waiting for Pivot to go up
                if (pivotTimer.milliseconds() >= Constant.PIVOT_UP_TIMER) {
                    setPivot(Constant.PIVOT_DOWN);
                }

                // Waiting for Pivot to come down + Fast leave to reduce runtime
                if (pivotTimer.milliseconds() < (artifactCount == 0 ? 5*Constant.PIVOT_DOWN_TIMER : Constant.PIVOT_DOWN_TIMER)) {
                    return;
                }

                // Next artifact or Exit
                nearestIndex = -1;
                if (artifactCount == 0) {
                    outtakeStage = -1;
                    Index = 1;
                    intakeDone = false;
                    setSpindexer(Constant.INTAKE_POS1);
                } else {
                    outtakeStage = 1;
                }
                break;
        }
    }
    public void update(String motif, boolean shooterReady) {
        handleAutonIntakeLogic();
        handleAutonOuttakeLogic(motif, shooterReady);
    }
    public void matrixUpdate(String motif,int line,boolean shooterReady,boolean shootOff){
        handleIntakeLogic();
        matrixOuttake(motif,line,shooterReady,shootOff);
    }

    public void matrixOuttake(String motif,int line,boolean shooterReady,boolean shootOff){


        if(shootOff){
            outtakeStage = -1;
        }else if(outtakeStage==-1){
            motifLine = shootMatrix[motif.equals("GPP")? 0:(motif.equals("PGP")? 1:2)][line-1];
            outtakeStage = 1;
        }

        switch (outtakeStage) {
            case 0: // Anti-stuck by going back to last outtake slot
                setSpindexer(lastPos);
                if (stateTimer.milliseconds() > 2 * Constant.ANTI_STUCK_TIMER) {
                    stateTimer.reset();
                    setSpindexer(nearestPos);

                    outtakeStage = 2;
                }
                break;


            case 1:
                spindexer1.setPosition(motifLine.charAt(0) == 'a' ?Constant.OUTTAKE_POS1:(motifLine.charAt(0) == 'b' ?Constant.OUTTAKE_POS2:Constant.OUTTAKE_POS3));

                if(stateTimer.milliseconds() > Constant.ANTI_STUCK_TIMER && shooterReady){
                    stateTimer.reset();
                    pivotTimer.reset();
                    outtakeStage = 2;
                }
                break;
            case 2:
                setPivot(Constant.PIVOT_UP);

                if (pivotTimer.milliseconds() >= Constant.PIVOT_UP_TIMER) {
                    setPivot(Constant.PIVOT_DOWN);
                    stateTimer.reset();
                    outtakeStage = 3;
                }
                break;

            case 3:

                if (pivotTimer.milliseconds() < (artifactCount == 0 ? 5*Constant.PIVOT_DOWN_TIMER : Constant.PIVOT_DOWN_TIMER)) {
                    return;
                }

                if(motifLine.isEmpty()){
                    outtakeStage = -1;
                    Index = 1;
                    intakeDone = false;
                    setSpindexer(Constant.INTAKE_POS1);
                }else {
                    motifLine = motifLine.substring(1);
                    outtakeStage = 1;
                }

                break;
        }
    }


    private void handleAutonIntakeLogic() {
        if (intakeStage == -1) {
            // Antistuck
            if (artifactCount == 3 && outtakeStage == -1) inverseIntake();
            else stopIntake();
            return;
        } else {
            intake.setPower(1);
        }

        switch (intakeStage) {
            case 1: // Color sensing
                if (artifactCount < 3 && colorSensor.blue() >= 150 && colorSensor.green() >= 150) {
                    String color = (colorSensor.blue() >= colorSensor.green()) ? "P" : "G";

                    // Record artifact into slots Array
                    slots[Index - 1] = new Artifact(color, getOuttakePos(Index));
                    artifactCount++;

                    // Next stage if less than 3 artifacts
                    intakeStage = (artifactCount < 3) ? 2 : -1;
                }
                break;

            case 2: // Getting index for next slot
                Index++;
                if (Index > 3) Index = 1; // module
                // Move to target index
                setSpindexer(getIntakePos(Index));
                stateTimer.reset();
                intakeStage = 3;
                break;

            case 3: // Waiting for motion
                targetTicks = getIntakeTick(Index);
                boolean inSlot = withinTarget(targetTicks, Constant.INTAKE_TICK_TOLERANCE);
                if (inSlot) {
                    // Start detecting again
                    intakeStage = 1;
                }
                break;
        }
    }
    public double getIntakePos(int i) { return (i==1) ? Constant.INTAKE_POS1 : (i==2) ? Constant.INTAKE_POS2 : Constant.INTAKE_POS3; }
    public double getOuttakePos(int i) { return (i==1) ? Constant.OUTTAKE_POS1 : (i==2) ? Constant.OUTTAKE_POS2 : Constant.OUTTAKE_POS3; }
    public int getOuttakeTick(int i) { return (i==1) ? Constant.OUTTAKE_POS1_TICK : (i==2) ? Constant.OUTTAKE_POS2_TICK : Constant.OUTTAKE_POS3_TICK; }
    public int getIntakeTick(int i) { return (i==1) ? Constant.INTAKE_POS1_TICK : (i==2) ? Constant.INTAKE_POS2_TICK : Constant.INTAKE_POS3_TICK; }
}

    private void handleAutonOuttakeLogic(String motif, boolean shooterReady) {

        switch (outtakeStage) {
            case 0: // Anti-stuck by going back to last outtake slot
                setSpindexer(lastPos);
                if (stateTimer.milliseconds() > 2 * Constant.ANTI_STUCK_TIMER) {
                    stateTimer.reset();
                    setSpindexer(nearestPos);

                    outtakeStage = 2;
                }
                break;

            case 1: // Selection Logic: 2 -> 1 -> 3

                int foundIndex = -1;
                targetColor = motif.substring(autonColor-1,autonColor);


                for (int i : priorityOrder) {
                    Artifact a = slots[i - 1];
                    // Skip loop if target NOT FOUND
                    if (a == null) continue;
                    if (targetColor.equals("ANY") || a.getColor().equals(targetColor)) {
                        // Target index FOUND
                        foundIndex = i;
                        break;
                    }
                }

                // For antistuck
                lastPos = nearestPos;

                if (foundIndex != -1) {
                    nearestIndex = foundIndex;
                    nearestPos = getOuttakePos(nearestIndex);
                    setSpindexer(nearestPos);
                    stateTimer.reset();
                    outtakeStage = 2;
                } else if (targetColor.equals("G") || targetColor.equals("P")) {
                    // If can't find specific color, fire closet slot
                    targetColor = "ANY";
                }
                break;

            case 2: // Waiting for Fire
                targetTicks = getOuttakeTick(nearestIndex);
                boolean inSlot = withinTarget(targetTicks, Constant.OUTTAKE_TICK_TOLERANCE);

                // Check Spindex pos and shooter RPM
                if (inSlot && shooterReady) {
                    slots[nearestIndex - 1] = null;
                    artifactCount--;
                    setPivot(Constant.PIVOT_UP);
                    pivotTimer.reset();
                    outtakeStage = 3;
                    autonColor = autonColor%3+1;
                    targetColor = "";
                } else if (stateTimer.milliseconds() > Constant.ANTI_STUCK_TIMER && !inSlot) {
                    stateTimer.reset();
                    outtakeStage = 0;
                }
                break;

            case 3: // Waiting for Pivot to go up
                if (pivotTimer.milliseconds() >= Constant.PIVOT_UP_TIMER) {
                    setPivot(Constant.PIVOT_DOWN);
                }

                // Waiting for Pivot to come down + Fast leave to reduce runtime
                if (pivotTimer.milliseconds() < (artifactCount == 0 ? 5*Constant.PIVOT_DOWN_TIMER : Constant.PIVOT_DOWN_TIMER)) {
                    return;
                }

                // Next artifact or Exit
                nearestIndex = -1;
                if (artifactCount == 0) {
                    outtakeStage = -1;
                    Index = 1;
                    intakeDone = false;
                    setSpindexer(Constant.INTAKE_POS1);
                } else {
                    outtakeStage = 1;
                }
                break;
        }
    }



    // --- Helper methods (DO NOT TOUCH) ---
    public class Artifact {
        private String color;
        private double position;

        public Artifact(String color, double position){
            this.color = color;
            this.position = position;
        }
        public String getColor() {
            return color;
        }
        public double getPosition(){
            return position;
        }
    }
    public void setSpindexer(double p) {
        spindexer1.setPosition(p);
        spindexer2.setPosition(p);
    }
    public void setPivot(double p) {
        leftPivot.setPosition(p + 0.015);
        rightPivot.setPosition(p);
    }
    private boolean withinTarget(int targetTicks, int tickTolerance) {
        currentTicks = spindexerEncoder.getCurrentPosition();
        return Math.abs(currentTicks - targetTicks) <= tickTolerance;
    }
    public double getIntakePos(int i) { return (i==1) ? Constant.INTAKE_POS1 : (i==2) ? Constant.INTAKE_POS2 : Constant.INTAKE_POS3; }
    public double getOuttakePos(int i) { return (i==1) ? Constant.OUTTAKE_POS1 : (i==2) ? Constant.OUTTAKE_POS2 : Constant.OUTTAKE_POS3; }
    public int getOuttakeTick(int i) { return (i==1) ? Constant.OUTTAKE_POS1_TICK : (i==2) ? Constant.OUTTAKE_POS2_TICK : Constant.OUTTAKE_POS3_TICK; }
    public int getIntakeTick(int i) { return (i==1) ? Constant.INTAKE_POS1_TICK : (i==2) ? Constant.INTAKE_POS2_TICK : Constant.INTAKE_POS3_TICK; }



}

