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


@Autonomous(name = "RED Test")
public class NearRedTest extends OpMode {

    // =========================================================================
    //  PATHS
    // =========================================================================

    public static class Paths {
        public PathChain MoveToShootPreload;
        public PathChain IntakeSecondRow;
        public PathChain ShootSecondRow;
        public PathChain MoveToGate;
        public PathChain MoveBack;
        public PathChain GateIntake;
        public PathChain ShootIntaked;
        public PathChain IntakeFirstRow;
        public PathChain ShootFirstRow;

        public Paths(Follower follower) {

            // Blue: (31,135)→(52,81.5) heading -90→-90
            // Mirror X; -90° mirrors to -90° (unchanged)
            MoveToShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(72, 135),   // 103-31
                            new Pose(51, 81.5)   // 103-52
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();

            // Blue: (52,81.5) curve → (15,60)  tangentHeading
            // Mirror X; tangent heading follows geometry automatically
            IntakeSecondRow = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(51, 81.5),           // 103-52
                            new Pose(53.62, 65.65),       // 103-49.38
                            new Pose(53.5, 56.5),         // 103-49.5
                            new Pose(53, 60),             // 103-50
                            new Pose(88, 60)              // 103-15
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // Blue: (15,60)→(56,82) tangent reversed
            // Mirror X; tangent & reversed stay
            ShootSecondRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88, 60),   // 103-15
                            new Pose(47, 82)    // 103-56
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Blue: (56,82) curve → (19,66.688)  heading -151°→180°
            // Mirror X; headings: 180°-(-151°)=-29°,  180°-180°=0°
            MoveToGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(47, 82),             // 103-56
                            new Pose(60.759, 71.732),     // 103-42.241
                            new Pose(84, 66.688)          // 103-19
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-29), Math.toRadians(0))
                    .build();

            // Blue: (19,66.688)→(19,64.081)  heading 180°→180°
            // Mirror X; headings: 180°-180°=0°
            MoveBack = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(84, 66.688),   // 103-19
                            new Pose(84, 64.081)    // 103-19
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Blue: (20.931,66.081) curve → (14,52)  heading 180°→130°
            // Mirror X; headings: 180°-180°=0°,  180°-130°=50°
            GateIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(82.069, 66.081),   // 103-20.931
                            new Pose(86.174, 57.282),   // 103-16.826
                            new Pose(89, 52)            // 103-14
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            // Blue: (14,54) curve → (56,82)  tangent reversed
            // Mirror X; tangent & reversed stay
            ShootIntaked = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(89, 54),           // 103-14
                            new Pose(69.023, 67.562),   // 103-33.977
                            new Pose(47, 82)            // 103-56
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Blue: (56,82)→(21,83)  heading 180°→180°
            // Mirror X; headings: 180°-180°=0°
            IntakeFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(47, 82),   // 103-56
                            new Pose(82, 83)    // 103-21
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Blue: (21,83)→(61.5,103)  heading 180°→180°
            // Mirror X; headings: 180°-180°=0°
            ShootFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82, 83),    // 103-21
                            new Pose(41.5, 103)  // 103-61.5
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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

    // Turret angles mirrored: 360 - blueAngle
    // Blue default 50° → 310°
    private double angle       = 310;
    private double odoDist     = 65;
    private String targetMotif = "Null";

    // Blue: (31, 135, -90°) → mirror X → (72, 135, -90°)
    public static final Pose START_POS = new Pose(72, 135, Math.toRadians(-90));

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── PRELOAD ───────────────────────────────────────────────────────

            case 0:
                follower.setMaxPower(1);
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
                // Blue 40° → mirror 320°
                angle = 320;
                if (spindexer.intakeStage == -1) {
                    spindexer.startOuttake();
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
                follower.followPath(paths.IntakeSecondRow, true);
                spindexer.startIntake();
                // Blue 345° → mirror 15°
                angle = 15;
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
                    spindexer.startOuttake();
                    setPathState(8);
                }
                break;

            case 8:
                if (spindexer.outtakeStage == -1) {
                    setPathState(10);
                }
                break;

            // ── GATE CYCLE 1 ─────────────────────────────────────────────────

            case 10:
                follower.followPath(paths.MoveToGate, true);
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    // Blue 350° → mirror 10°
                    angle = 10;
                    follower.followPath(paths.GateIntake, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    spindexer.startOuttake();
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
                follower.followPath(paths.MoveToGate, true);
                setPathState(21);
                break;

            case 21:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateIntake, true);
                    setPathState(23);
                }
                break;

            case 23:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(24);
                }
                break;

            case 24:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    // Blue 350° → mirror 10°
                    angle = 10;
                    spindexer.startOuttake();
                    setPathState(25);
                }
                break;

            case 25:
                if (spindexer.outtakeStage == -1) {
                    setPathState(26);
                }
                break;

            // ── GATE CYCLE 3 ─────────────────────────────────────────────────

            case 26:
                follower.followPath(paths.MoveToGate, true);
                setPathState(27);
                break;

            case 27:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(28);
                }
                break;

            case 28:
                if (!follower.isBusy()) {
                    // Blue 350° → mirror 10°
                    angle = 10;
                    follower.followPath(paths.GateIntake, true);
                    setPathState(29);
                }
                break;

            case 29:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(30);
                }
                break;

            case 30:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(31);
                }
                break;

            case 31:
                if (spindexer.outtakeStage == -1) {
                    setPathState(32);
                }
                break;

            // ── FIRST ROW ────────────────────────────────────────────────────

            case 32:
                odoDist = 60;
                spindexer.startIntake();
                follower.followPath(paths.IntakeFirstRow, true);
                setPathState(33);
                break;

            case 33:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootFirstRow, true);
                    // Blue 300° → mirror 60°
                    angle = 60;
                    setPathState(34);
                }
                break;

            case 34:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(35);
                }
                break;

            case 35:
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
        Constant.ALLIANCE = "RED";

        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shooter   = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        spindexer.setSpindexer(Constant.INTAKE_POS1);
        // Blue init: 0.3 → mirror: 1.0 - 0.3 = 0.7
        shooter.setTurretPosition(0.7);

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

        // Blue pre-motif: 40° → mirror 320°
        if (targetMotif.equals("Null")) {
            shooter.updateTurret(320, 0);
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