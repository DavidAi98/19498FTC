package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Spindexer {
    public int brightness;
    public float[] HSV = new float[3];
    public boolean noSort = false;
    public DcMotor spindexerEncoder, intake;
    public Servo spindexer1, spindexer2, leftPivot, rightPivot;

    public RevColorSensorV3 colorSensor1;
    public RevColorSensorV3 colorSensor2;
    private boolean inSlot;

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
    public String targetColor = "NaN";

    private static final int[] priorityOrder = {2, 1, 3};
    public int autonColor = 1;
    private boolean colorDetected;

    public int sensorInUse = 1;
    public boolean onStart = true;

    public Spindexer(@NonNull HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "IntakeMotor");
        spindexer1 = hwMap.get(Servo.class, "spindexer1");
        spindexer2 = hwMap.get(Servo.class, "spindexer2");
        leftPivot = hwMap.get(Servo.class, "LeftPivot");
        rightPivot = hwMap.get(Servo.class, "RightPivot");
        colorSensor1 = hwMap.get(RevColorSensorV3.class, "colorSensor1");
        colorSensor2 = hwMap.get(RevColorSensorV3.class, "colorSensor2");
        spindexerEncoder = hwMap.get(DcMotor.class, "SpindexerEncoder");
        spindexerEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftPivot.setDirection(Servo.Direction.REVERSE);
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
                intake.setPower(0);
                encoderResetDone = true;
            } else if (resetTimer.milliseconds() >= Constant.ANTI_STUCK_TIMER){
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        }
    }
    public void startIntake() {
        if (outtakeStage == -1 && artifactCount < 3) {
            setSpindexer(getIntakePos(Index));
            intakeStage = 1;
        }
    }

    public void stopIntake() {
        intake.setPower(0);
        intakeStage = -1;
    }

    public void inverseIntake() {
        if (!intakeDone) {
            intakeStage = -1;
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

    public void stopOuttake() {
        outtakeStage = -1;
    }


    private void handleIntakeLogic(boolean skipSlot) {
        if (intakeStage == -1) {
            if (artifactCount == 3 && outtakeStage == -1) inverseIntake();
            else stopIntake();
            return;
        } else {
            intake.setPower(1);
        }
        boolean sensor1dc = colorSensor1.getDistance(DistanceUnit.MM) == 152.4;
        boolean sensor2dc = colorSensor2.getDistance(DistanceUnit.MM) == 152.4;


        if (sensor1dc && sensor2dc) {
            sensorInUse = -1;
        } else if (sensor1dc) {
            sensorInUse = 2;
        } else if (sensor2dc) {
            sensorInUse = 1;
        }

        switch (intakeStage) {
            case 1:
                targetTicks = getIntakeTick(Index);
                inSlot = withinTarget(targetTicks, Constant.INTAKE_TICK_TOLERANCE);
                if (!inSlot || artifactCount == 3) {
                    return;
                }

                if (sensorInUse == 1) {
                    colorDetected = artifactCount < 3 && colorSensor1.blue() + colorSensor1.green() > 175;
                } else if (sensorInUse == 2) {
                    colorDetected = artifactCount < 3 && colorSensor2.blue() + colorSensor2.green() > 175;
                }

                if (colorDetected || sensorInUse == -1 || skipSlot) {
                    if (sensorInUse == 2) {
                        color = returnColor(colorSensor2.red(),colorSensor2.green(),colorSensor2.blue());
                    } else if (sensorInUse == 1) {
                        color = returnColor(colorSensor1.red(),colorSensor1.green(),colorSensor1.blue());
                    } else if (sensorInUse == -1){
                        color =  "P";
                    } else {
                        return;
                    }

                    if (!color.equals("NaN")) {
                        slots[Index - 1] = new Artifact(color, getOuttakePos(Index));
                        artifactCount++;
                        intakeStage = (artifactCount < 3) ? 2 : -1;
                    }
                }
                break;

            case 2:
                Index++;
                if (Index > 3) Index = 1;
                setSpindexer(getIntakePos(Index));
                stateTimer.reset();
                intakeStage = 3;
                break;

            case 3:
                targetTicks = getIntakeTick(Index);
                inSlot = withinTarget(targetTicks, Constant.INTAKE_TICK_TOLERANCE);
                if (inSlot) {
                    // Start detecting again
                    intakeStage = 1;
                } else if (stateTimer.milliseconds() > Constant.ANTI_STUCK_TIMER) {
                    resetTimer.reset();
                    encoderResetDone = false;
                    intake.setPower(-1);
                }
                break;
        }
    }


    private void handleOuttakeLogic(boolean fireButton, boolean shooterReady, boolean purpleButton, boolean greenButton) {
        if (outtakeStage == -1 || (artifactCount == 0 && outtakeStage != 3)) {
            outtakeStage = -1;
            return;
        }

        switch (outtakeStage) {
            case 0:
                setSpindexer(lastPos);
                if (stateTimer.milliseconds() > 2 * Constant.ANTI_STUCK_TIMER) {
                    stateTimer.reset();
                    setSpindexer(nearestPos);
                    outtakeStage = 2;
                }
                break;

            case 1:
                if (fireButton)        targetColor = "ANY";
                else if (purpleButton) targetColor = "P";
                else if (greenButton)  targetColor = "G";
                if (targetColor.equals("NaN")) return;

                int foundIndex = -1;
                for (int i : priorityOrder) {
                    Artifact a = slots[i - 1];
                    if (a == null) continue;
                    if (targetColor.equals("ANY") || a.getColor().equals(targetColor)) {
                        foundIndex = i;
                        break;
                    }
                }
                lastPos = nearestPos;
                if (foundIndex != -1) {
                    nearestIndex = foundIndex;
                    nearestPos = getOuttakePos(nearestIndex);
                    lastPos = nearestPos;
                    setSpindexer(nearestPos);
                    stateTimer.reset();
                    outtakeStage = 2;
                } else if (targetColor.equals("G") || targetColor.equals("P")) {
                    targetColor = "ANY";
                }
                break;

            case 2:
                targetTicks = getOuttakeTick(nearestIndex);
                boolean inSlot = withinTarget(targetTicks, Constant.OUTTAKE_TICK_TOLERANCE);
                boolean notStuck = withinTarget(targetTicks, Constant.OUTTAKE_TICK_TOLERANCE-200);
                if (inSlot && shooterReady) {
                    slots[nearestIndex - 1] = null;
                    artifactCount--;
                    setPivot(Constant.PIVOT_UP);
                    pivotTimer.reset();
                    outtakeStage = 3;
                    targetColor = "NaN";
                } else if (stateTimer.milliseconds() > Constant.ANTI_STUCK_TIMER && !notStuck) {
                    stateTimer.reset();
                    outtakeStage = 0;
                }
                break;

            case 3:
                if (pivotTimer.milliseconds() >= Constant.PIVOT_UP_TIMER) {
                    setPivot(Constant.PIVOT_DOWN);
                }
                if (pivotTimer.milliseconds() < (artifactCount == 0 ? 2 * Constant.PIVOT_DOWN_TIMER : Constant.PIVOT_DOWN_TIMER)) {
                    return;
                }
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
        if (encoderResetDone) {
            handleAutonIntakeLogic();
            handleAutonOuttakeLogic(motif, shooterReady);
        } else {
            intakeStage = -1;
            outtakeStage = -1;
            artifactCount = 0;
            Index = 1;
            setSpindexer(Constant.INTAKE_POS1);
            if (resetTimer.milliseconds() >= 2 * Constant.ANTI_STUCK_TIMER) {
                spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spindexerEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake.setPower(0);
                encoderResetDone = true;
            } else if (resetTimer.milliseconds() >= Constant.ANTI_STUCK_TIMER) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        }
    }
    private void handleAutonIntakeLogic() {
        if (intakeStage == -1) {
            if (artifactCount == 3 && outtakeStage == -1) inverseIntake();
            else stopIntake();
            return;
        } else {
            intake.setPower(1);
        }

        double sensor1dist = colorSensor1.getDistance(DistanceUnit.MM);
        double sensor2dist = colorSensor2.getDistance(DistanceUnit.MM);
        if ((sensor2dist == 152.4) && (sensor1dist == 152.4)) {
            sensorInUse = -1;
        } else if (sensor1dist == 152.4) {
            sensorInUse = 2;
        } else if (sensor2dist == 152.4) {
            sensorInUse = 1;
        }

        switch (intakeStage) {
            case 1:
                targetTicks = getIntakeTick(Index);
                inSlot = withinTarget(targetTicks, Constant.INTAKE_TICK_TOLERANCE);
                if (!inSlot || artifactCount == 3) {
                    return;
                }

                if (sensorInUse == 1) {
                    colorDetected = artifactCount < 3 && colorSensor1.blue() + colorSensor1.green() > 175;
                } else if (sensorInUse == 2) {
                    colorDetected = artifactCount < 3 && colorSensor2.blue() + colorSensor2.green() > 175;
                }

                if (colorDetected || sensorInUse == -1) {
                    if (sensorInUse == 2) {
                        color = returnColor(colorSensor2.red(),colorSensor2.green(),colorSensor2.blue());
                    } else if (sensorInUse == 1) {
                        color = returnColor(colorSensor1.red(),colorSensor1.green(),colorSensor1.blue());
                    } else if (sensorInUse == -1){
                        color =  "P";
                    } else {
                        return;
                    }

                    if (!color.equals("NaN")) {
                        slots[Index - 1] = new Artifact(color, getOuttakePos(Index));
                        artifactCount++;
                        intakeStage = (artifactCount < 3) ? 2 : -1;
                    }
                }
                break;

            case 2:
                if (artifactCount >= 3) {
                    intakeStage = -1; // Stop here if full
                } else {
                    Index++;
                    if (Index > 3) Index = 1;
                    setSpindexer(getIntakePos(Index));
                    stateTimer.reset();
                    intakeStage = 3;
                }
                break;

            case 3:
                targetTicks = getIntakeTick(Index);
                boolean inSlot = withinTarget(targetTicks, Constant.INTAKE_TICK_TOLERANCE);
                if (inSlot) {
                    intakeStage = 1;
                } else if (stateTimer.milliseconds() > Constant.ANTI_STUCK_TIMER) {
                    resetTimer.reset();
                    encoderResetDone = false;
                    intake.setPower(-1);
                }
                break;
        }
    }


    private void handleAutonOuttakeLogic(String motif, boolean shooterReady) {
        if (artifactCount == 0 && outtakeStage != 3) {
            outtakeStage = -1;
            return;
        }

        switch (outtakeStage) {
            case 0:
                setSpindexer(lastPos);
                if (stateTimer.milliseconds() > 2 * Constant.ANTI_STUCK_TIMER) {
                    stateTimer.reset();
                    setSpindexer(nearestPos);
                    intake.setPower(0);
                    outtakeStage = 2;
                }
                break;

            case 1:
                int foundIndex = -1;
                if (noSort) {
                    targetColor = "ANY";
                } else if (targetColor.equals("NaN")) {
                    targetColor = motif.substring(autonColor - 1, autonColor);
                }
                for (int i : priorityOrder) {
                    Artifact a = slots[i - 1];
                    if (a == null) continue;
                    if (targetColor.equals("ANY") || a.getColor().equals(targetColor)) {
                        foundIndex = i;
                        break;
                    }
                }
                lastPos = nearestPos;
                if (foundIndex != -1) {
                    nearestIndex = foundIndex;
                    nearestPos = getOuttakePos(nearestIndex);
                    setSpindexer(nearestPos);
                    stateTimer.reset();
                    outtakeStage = 2;
                } else if (targetColor.equals("G") || targetColor.equals("P")) {
                    targetColor = "ANY";
                }
                break;

            case 2:
                targetTicks = getOuttakeTick(nearestIndex);
                boolean inSlot = withinTarget(targetTicks, Constant.OUTTAKE_TICK_TOLERANCE);
                if (inSlot && shooterReady) {
                    slots[nearestIndex - 1] = null;
                    artifactCount--;
                    setPivot(Constant.PIVOT_UP);
                    pivotTimer.reset();
                    outtakeStage = 3;
                    autonColor = autonColor % 3 + 1;
                    targetColor = "NaN";
                } else if (stateTimer.milliseconds() > Constant.ANTI_STUCK_TIMER && !inSlot) {
                    stateTimer.reset();
                    outtakeStage = 0;
                }
                break;

            case 3:
                if (pivotTimer.milliseconds() >= Constant.PIVOT_UP_TIMER) {
                    setPivot(Constant.PIVOT_DOWN);
                }
                if (pivotTimer.milliseconds() < (artifactCount == 0 ? 2 * Constant.PIVOT_DOWN_TIMER : Constant.PIVOT_DOWN_TIMER)) {
                    return;
                }
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


    public class Artifact {
        private String color;
        private double position;

        public Artifact(String color, double position) {
            this.color    = color;
            this.position = position;
        }

        public String getColor()    { return color;    }
        public double getPosition() { return position; }
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

    public String returnColor(float r,float g,float b) {
        if (g > b) {
            return "G";
        } else if (b > g) {
            return "P";
        }
        return "NaN";
    }
}