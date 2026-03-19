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

@Autonomous(name = "BLUE Test")
public class NearBlueTest extends OpMode {

    // =========================================================================
    //  PATHS
    // =========================================================================

    public static class Paths {
        public PathChain MoveToShootPreload;
//        public PathChain MoveToSecondRow;
        public PathChain IntakeSecondRow;
        public PathChain ShootSecondRow;
        public PathChain MoveToGate;
        public PathChain MoveBack;
        public PathChain GateIntake;
        public PathChain ShootIntaked;
        public PathChain IntakeFirstRow;
        public PathChain ShootFirstRow;

        public Paths(Follower follower) {
            MoveToShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(31, 135),
                            new Pose(52, 81.5)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();

//            MoveToSecondRow = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(52, 81.5),
//                            new Pose(54, 60),
//                            new Pose(48, 60)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
//                    .build();

            IntakeSecondRow = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(52, 81.5),
                            new Pose(52, 66),
                            new Pose(49.5, 56.5),
                            new Pose(50, 60),
                            new Pose(15, 60)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            ShootSecondRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(15, 60),
                            new Pose(56, 82)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            MoveToGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56, 82),
                            new Pose(42.241, 71.732),
                            new Pose(19, 66.688)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-151), Math.toRadians(180))
                    .build();

            MoveBack = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(19, 66.688),
                            new Pose(19, 64.081)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            GateIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(20.931, 66.081),
                            new Pose(16.826, 57.282),
                            new Pose(14, 52)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            ShootIntaked = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(14, 54),
                            new Pose(33.977, 67.562),
                            new Pose(56, 82)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            IntakeFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 82),
                            new Pose(21, 83)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ShootFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(21, 83),
                            new Pose(61.5, 103)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
    private double odoDist     = 65;
    private String targetMotif = "Null";

    public static final Pose START_POS = new Pose(31, 135, Math.toRadians(-90));

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── PRELOAD ───────────────────────────────────────────────────────

            // Move to shooting position; start intake to stage preload
            case 0:
                follower.followPath(paths.MoveToShootPreload, true);
                spindexer.startIntake();
                setPathState(1);
                break;

            // Wait for arrival + motif lock
            case 1:
                if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                    targetMotif = "PPG";
                    setPathState(2);
                } else if (!follower.isBusy() && !targetMotif.equals("Null")) {
                    setPathState(2);
                }
                break;

            // Wait for preload to stage then fire
            case 2:
                angle = 50;
                if (spindexer.intakeStage == -1) { //&& pathTimer.getElapsedTimeSeconds() > 0.1) {
                    spindexer.startOuttake();
                    setPathState(3);
                }
                break;

            // Wait for preload shot to clear
            case 3:
                if (spindexer.outtakeStage == -1) {
                    setPathState(4);
                }
                break;

            // ── SECOND ROW ───────────────────────────────────────────────────

            // Curve down to front of second row; start intake
            case 4:
                follower.followPath(paths.IntakeSecondRow, true);
                spindexer.startIntake();
                angle = 345;
                setPathState(6);
                break;

            // Wait to arrive at second row start, then sweep
//            case 5:
//                if (!follower.isBusy()) {
//                    angle = 345;
//                    follower.followPath(paths.IntakeSecondRow, true);
//                    setPathState(6);
//                }
//                break;

            // Sweep done OR intake full; stop intake and head back to shoot
            case 6:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootSecondRow, true);
                    setPathState(7);
                }
                break;

            // Wait to arrive at shooting position then fire
            case 7:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(8);
                }
                break;

            // Wait for second row shot to clear
            case 8:
                if (spindexer.outtakeStage == -1) {
                    setPathState(10);
                }
                break;

            // ── GATE CYCLE 1 ─────────────────────────────────────────────────

            // Head to gate (no intake yet — start at MoveBack)
            case 10:
                follower.followPath(paths.MoveToGate, true);
                setPathState(11);
                break;

            // Arrived at gate; start intake then nudge back
            case 11:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(12);
                }
                break;

            // Nudge done; sweep gate arc
            case 12:
                if (!follower.isBusy()) {
                    angle = 350;
                    follower.followPath(paths.GateIntake, true);
                    setPathState(13);
                }
                break;

            // Intake until full or 2s timeout; stop intake and head to shoot
            case 13:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(14);
                }
                break;

            // Wait to arrive then fire gate cycle 1
            case 14:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(15);
                }
                break;

            // Wait for gate cycle 1 shot to clear
            case 15:
                if (spindexer.outtakeStage == -1) {
                    setPathState(20);
                }
                break;

            // ── GATE CYCLE 2 ─────────────────────────────────────────────────

            // Head to gate (no intake yet)
            case 20:
                follower.followPath(paths.MoveToGate, true);
                setPathState(21);
                break;

            // Arrived; start intake then nudge back
            case 21:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(22);
                }
                break;

            // Nudge done; sweep gate arc
            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateIntake, true);
                    setPathState(23);
                }
                break;

            // Intake until full or 2s timeout; stop intake and head to shoot
            case 23:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(24);
                }
                break;

            // Wait to arrive then fire gate cycle 2
            case 24:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    angle = 350;
                    spindexer.startOuttake();
                    setPathState(25);
                }
                break;

            // Wait for gate cycle 2 shot to clear
            case 25:
                if (spindexer.outtakeStage == -1) {
                    setPathState(30);
                }
                break;

            // ── FIRST ROW ────────────────────────────────────────────────────

            // Sweep across first row with intake on
            case 30:
                odoDist = 60;
                spindexer.startIntake();
                follower.followPath(paths.IntakeFirstRow, true);
                setPathState(31);
                break;

            // Wait for path done or intake full; stop intake and head to shoot
            case 31:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootFirstRow, true);
                    setPathState(32);
                }
                break;

            // Wait to arrive then fire first row
            case 32:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    angle = 300;
                    spindexer.startOuttake();
                    setPathState(33);
                }
                break;

            // Wait for first row shot to clear
            case 33:
                if (spindexer.outtakeStage == -1) {
                    setPathState(99);
                }
                break;

            // ── DONE ─────────────────────────────────────────────────────────
            case 99:
                break;
        }
    }

    // =========================================================================
    //  LIFECYCLE
    // =========================================================================

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

        // Hold at 40° facing goal until motif is seen, then use per-shot angle
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