package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE 18")
public class Blue18 extends OpMode {

    // =========================================================================
    //  PATHS
    // =========================================================================

    public class Paths {
        public PathChain MoveToShootPreload;
        public PathChain MoveToSecondRow;
        public PathChain IntakeSecondRow;
        public PathChain ShootSecondRow;
//        public PathChain MoveToGate;
//        public PathChain MoveBack;
        public PathChain GateIntake;
        public PathChain ShootIntaked;
        public PathChain IntakeFirstRow;
        public PathChain ShootFirstRow;

        public Paths(Follower follower) {
            MoveToShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(31, 135),
                            new Pose(58, 76)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .addParametricCallback(0.75, () -> spindexer.stopIntake())
                    .addParametricCallback(0.85, () -> spindexer.startOuttake())
                    .build();

            MoveToSecondRow = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58, 76),
                            new Pose(58, 68),
                            new Pose(47, 61),
                            new Pose(13.000, 60.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            ShootSecondRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13, 60),
                            new Pose(56, 84)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

//            MoveToGate = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(56, 83),
//                            new Pose(42.241, 71.732),
//                            new Pose(23, 67.688)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(-151), Math.toRadians(180))
//                    .build();
//
//            MoveBack = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(23, 67.688),
//                            new Pose(22, 64.081)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();

            // startIntake at 0.1 — fires just after movement begins,
            // giving outtake from previous shot time to finish during the drive
            GateIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56, 84),
                            new Pose(35.000, 55),
                            new Pose(13.5, 59)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(138))
                    .addParametricCallback(0.1, () -> spindexer.startIntake())
                    .build();

            // t=0.6 → stopIntake  (no new balls staging mid-shot)
            // t=0.75 → startOuttake  (fire sequence starts while still driving)
            ShootIntaked = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(13.5, 59),
                            new Pose(33.977, 67.562),
                            new Pose(56, 84)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .setBrakingStrength(1.6)
                    .setBrakingStart(0.25)
                    .addParametricCallback(0.75, () -> spindexer.stopIntake())
                    .addParametricCallback(0.85, () -> spindexer.startOuttake())
                    .build();

            IntakeFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 84),
                            new Pose(21, 85)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ShootFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(21, 85),
                            new Pose(51, 115)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.75, () -> spindexer.stopIntake())
                    .addParametricCallback(0.85, () -> spindexer.startOuttake())
                    .build();
        }
    }

    // =========================================================================
    //  FIELDS
    // =========================================================================

    private Follower  follower;
    private Paths     paths;
    private Timer     pathTimer, opmodeTimer;
    private int       pathState;

    private Shooter   shooter;
    private Spindexer spindexer;

    private double angle       = 50;
    private double odoDist     = 64;
    private String targetMotif = "Null";

    public static final Pose START_POS = new Pose(31, 135, Math.toRadians(-90));

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── PRELOAD ───────────────────────────────────────────────────────

            case 0:
                follower.setMaxPower(1.0);
                follower.followPath(paths.MoveToShootPreload, true);
                spindexer.startIntake();
                setPathState(1);
                break;

            case 1:
                if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                    targetMotif = "PPG";
                    setPathState(2);
                } else if (!follower.isBusy() && !targetMotif.equals("Null")) {
                    setPathState(2);
                }
                break;

            case 2:
                angle = 50;
                if (spindexer.intakeStage == -1) {
                    setPathState(3);
                }
                break;

            case 3:
                if (spindexer.outtakeStage == -1) {
                    setPathState(4);
                }
                break;

            // ── SECOND ROW ───────────────────────────────────────────────────

            case 4:
                follower.followPath(paths.MoveToSecondRow, true);
                spindexer.startIntake();
                angle = 345;
                odoDist = 60;
                setPathState(6);
                break;

            case 6:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootSecondRow, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    setPathState(8);
                }
                break;

            case 8:
                if (spindexer.outtakeStage == -1) {
                    setPathState(12);
                }
                break;

            // ── GATE CYCLE 1 ─────────────────────────────────────────────────

            case 12:
                    angle = 345;
                    spindexer.startIntake();
                    follower.followPath(paths.GateIntake, true);
                    setPathState(13);
                break;

            // Callbacks on ShootIntaked handle stopIntake + startOuttake
            case 13:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(14);
                }
                break;

            // Just wait for path to finish; outtake already started via callback
            case 14:
                if (!follower.isBusy()) {
                    setPathState(15);
                }
                break;

            case 15:
                if (spindexer.outtakeStage == -1) {
                    setPathState(20);
                }
                break;

            // ── GATE CYCLE 2 ─────────────────────────────────────────────────

            case 20:
                angle = 345;
                spindexer.startIntake();
                follower.followPath(paths.GateIntake, true);
                setPathState(21);
                break;

            // Callbacks on ShootIntaked handle stopIntake + startOuttake
            case 21:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(22);
                }
                break;

            // Just wait for path to finish; outtake already started via callback
            case 22:
                if (!follower.isBusy()) {
                    setPathState(23);
                }
                break;

            case 23:
                if (spindexer.outtakeStage == -1) {
                    setPathState(24);
                }
                break;

            // ── GATE CYCLE 3 ─────────────────────────────────────────────────

            case 24:
                angle = 345;
                spindexer.startIntake();
                follower.followPath(paths.GateIntake, true);
                setPathState(25);
                break;

            // Callbacks on ShootIntaked handle stopIntake + startOuttake
            case 25:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(26);
                }
                break;

            // Just wait for path to finish; outtake already started via callback
            case 26:
                if (!follower.isBusy()) {
                    setPathState(27);
                }
                break;

            case 28:
                if (spindexer.outtakeStage == -1) {
                    setPathState(29);
                }
                break;

            // ── FIRST ROW ────────────────────────────────────────────────────

            case 30:
                spindexer.startIntake();
                follower.followPath(paths.IntakeFirstRow, true);
                setPathState(33);
                break;

            case 33:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootFirstRow, true);
                    angle = 340;
                    odoDist = 20;
                    setPathState(34);
                }
                break;

            case 34:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    setPathState(35);
                }
                break;

            case 35:
                if (spindexer.outtakeStage == -1) {
                    setPathState(99);
                }
                break;

            case 99:
                break;
        }
    }


    @Override
    public void init() {
        Constant.ALLIANCE = "BLUE";

        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shooter   = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        spindexer.setSpindexer(Constant.INTAKE_POS1);
        shooter.setTurretPosition(0.3);

        follower = Constants.createFollower(hardwareMap);
        paths    = new Paths(follower);
        follower.setStartingPose(START_POS);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // Poll every loop — never misses the motif
        if (targetMotif.equals("Null")) {
            targetMotif = shooter.detectMotif();
        }

        shooter.updateShootingParams(odoDist, 20, spindexer.outtakeStage != -1);

        if (targetMotif.equals("Null")) {
            shooter.updateTurret(100, 0);
        } else {
            shooter.updateTurret(angle, 0);
        }

        shooter.runShooter(spindexer.outtakeStage != -1);
        spindexer.update(targetMotif, shooter.isReady());

        autonomousPathUpdate();

        // Slot visual
        StringBuilder slotVisual = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if      (spindexer.slots[i] == null)                    slotVisual.append("⚪ ");
            else if (spindexer.slots[i].getColor().equals("P"))     slotVisual.append("\uD83D\uDFE3 ");
            else if (spindexer.slots[i].getColor().equals("G"))     slotVisual.append("\uD83D\uDFE2 ");
        }

        // Save pose for TeleOp hand-off
        Pose p = follower.getPose();
        Constant.AUTON_LAST_X           = 103 - p.getX();
        Constant.AUTON_LAST_Y           =   3 - p.getY();
        Constant.AUTON_LAST_HEADING_RAD = p.getHeading() - Math.PI;
        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);

        telemetry.addData("Encoder Reset",  spindexer.encoderResetDone);
        telemetry.addData("Slots",         slotVisual.toString());
        telemetry.addData("Path State",    pathState);
        telemetry.addData("Motif",         targetMotif);
        telemetry.addData("Turret Angle",  angle);
        telemetry.addData("Intake Stage",  spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
        telemetry.addData("Velo Error",    "%.1f",
                shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());
        telemetry.addData("Target Color",  spindexer.targetColor);
        telemetry.addData("Max Power",     follower.getMaxPowerScaling());
        telemetry.addData("Heading",       follower.getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}