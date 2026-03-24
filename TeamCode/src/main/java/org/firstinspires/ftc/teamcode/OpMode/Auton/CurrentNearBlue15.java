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

@Autonomous(name = "BLUE \uD83D\uDD35 15 Solo (Sorted)")
public class CurrentNearBlue15 extends OpMode {

    // =========================================================================
    //  PATHS
    // =========================================================================

    public class Paths {
        public PathChain MoveToShootPreload;
        public PathChain IntakeSecondRow;
        public PathChain SecondRowToGate;
        public PathChain GateToShoot;
        public PathChain ShootPosToGate;
        public PathChain GateIntake;
        public PathChain ShootGate;
        public PathChain MoveToThirdRow;
        public PathChain ShootThirdRow;
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

            IntakeSecondRow = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(52.000, 81.500),
                            new Pose(57.604, 57.275),
                            new Pose(31.362, 62.304),
                            new Pose(16.000, 60.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // Second row end → gate, no intake
            SecondRowToGate = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(16, 60),
                            new Pose(22, 69.688)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Gate → shoot, fires 2nd row in pattern at t=0.7
            GateToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(22, 69.688),
                            new Pose(42.241, 71.732),
                            new Pose(56, 83)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.7, () -> spindexer.startOuttake())
                    .build();

            // Shoot pos → gate approach
            ShootPosToGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56.000, 83.000),
                            new Pose(45.55334866828087, 67.7223147699758),
                            new Pose(22, 66.688)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(150))
                    .build();

            // Gate sweep — intake on the whole time
            GateIntake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22, 66.688),
                            new Pose(14, 51)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(140))
                    .build();

            // Gate sweep end → shoot, fires gate balls in pattern
            ShootGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(14, 51 ),
                            new Pose(33.977, 67.562),
                            new Pose(56, 83)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .setBrakingStrength(1.6)
                    .setBrakingStart(0.25)
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
                    .build();

            // Shoot pos → third row sweep — intake on
            MoveToThirdRow = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56, 83),
                            new Pose(56.38366101694915, 48.03529055690071),
                            new Pose(56.117447941888635, 33.57518644067797),
                            new Pose(13.128, 36.745)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // Third row end → shoot, fires 3rd row in pattern
            ShootThirdRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13.128, 36.745),
                            new Pose(56, 83)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
                    .build();

            // Shoot pos → first row sweep — intake on
            IntakeFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 83),
                            new Pose(21, 85)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // First row end → elevated shoot pos, fires 1st row in pattern
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

    private double angle       = 45;
    private double odoDist     = 62;
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
                    spindexer.targetColor = "ANY";
                    spindexer.startOuttake();
                    setPathState(3);
                }
                break;

            case 3:
                if (spindexer.outtakeStage == -1) {
                    spindexer.autonColor = 1;
                    setPathState(10);
                }
                break;

            // SECOND ROW

            // Sweep second row with intake on
            case 10:
                follower.followPath(paths.IntakeSecondRow, true);
                spindexer.startIntake();
                angle = 350;
                odoDist = 55;
                setPathState(11);
                break;

            // Done sweeping; stop intake, transit to gate
            case 11:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.SecondRowToGate, true);
                    setPathState(12);
                }
                break;

            // Arrived at gate; wait 1s then drive to shoot — GateToShoot fires 2nd row
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    spindexer.stopIntake();
                    angle = 352;
                    odoDist = 64;
                    follower.followPath(paths.GateToShoot, true);
                    setPathState(13);
                }
                break;

            // Wait for path + 2nd row balls fired
            case 13:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(20);
                }
                break;

            // ── GATE INTAKE ───────────────────────────────────────────────────

            // Drive to gate
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                follower.followPath(paths.ShootPosToGate, true);
                setPathState(21);
                }
                break;

            // Arrived at gate; start intake immediately then sweep
            case 21:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    angle = 345;
                    follower.followPath(paths.GateIntake, true);
                    setPathState(22);
                }
                break;

            // Sweeping gate; full or 1.75s — ShootGate callbacks stop + fire pattern
            case 22:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootGate, true);
                    setPathState(23);
                }
                break;

            // Wait for path + gate balls fired
            case 23:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(30);
                }
                break;

            // ── THIRD ROW ─────────────────────────────────────────────────────

            // Sweep third row with intake on; ShootThirdRow callbacks fire in pattern
            case 30:
                spindexer.startIntake();
                follower.followPath(paths.MoveToThirdRow, true);
                angle = 5;
                odoDist = 70;
                setPathState(31);
                break;

            // Done sweeping; drive to shoot
            case 31:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootThirdRow, true);
                    setPathState(32);
                }
                break;

            // Wait for path + 3rd row balls fired
            case 32:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(40);
                }
                break;

            // ── FIRST ROW ─────────────────────────────────────────────────────

            // Sweep first row with intake on; ShootFirstRow callbacks fire in pattern
            case 40:
                spindexer.startIntake();
                follower.followPath(paths.IntakeFirstRow, true);
                angle = 335;
                odoDist = 27;
                setPathState(41);
                break;

            // Done sweeping; drive to elevated shoot pos
            case 41:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootFirstRow, true);
                    setPathState(42);
                }
                break;

            // Wait for path + 1st row balls fired
            case 42:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
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
        spindexer.noSort = false;
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

        StringBuilder slotVisual = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if      (spindexer.slots[i] == null)                    slotVisual.append("⚪ ");
            else if (spindexer.slots[i].getColor().equals("P"))     slotVisual.append("\uD83D\uDFE3 ");
            else if (spindexer.slots[i].getColor().equals("G"))     slotVisual.append("\uD83D\uDFE2 ");
        }

        Pose p = follower.getPose();
        Constant.AUTON_LAST_X           = 103 - p.getX();
        Constant.AUTON_LAST_Y           =   3 - p.getY();
        Constant.AUTON_LAST_HEADING_RAD = p.getHeading() - Math.PI;
        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);

        telemetry.addData("Slots",         slotVisual.toString());
        telemetry.addData("Path State",    pathState);
        telemetry.addData("Motif",         targetMotif);
        telemetry.addData("Turret Angle",  angle);
        telemetry.addData("Odo Dist",      odoDist);
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