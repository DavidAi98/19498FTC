package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Current blue 15")
public class CurrentNearBlue15 extends OpMode {


    public class Paths {
        public PathChain MoveToShootPreload;
        public PathChain IntakeSecondRow;
        public PathChain SecondRowToGate;
        public PathChain GateToShoot;
        public PathChain GateIntake;
        public PathChain ShootGate;
        public PathChain MoveToThirdRow;
        public PathChain ShootThirdRow;
        public PathChain IntakeFirstRow;
        public PathChain ShootFirstRow;

        public Paths(Follower follower) {
            MoveToShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(31.000, 135.000),

                                    new Pose(57, 84)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                    .build();

            IntakeSecondRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57, 84),
                                    new Pose(58.000, 62),
                                    new Pose(56.000, 60.000),
                                    new Pose(50, 59.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(50, 59.000),

                                    new Pose(13.000, 59.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SecondRowToGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.000, 59.000),
                                    new Pose(27.988, 53.907),
                                    new Pose(20, 73)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(183))
                    .build();

            GateToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(20, 73),
                                    new Pose(30, 70),
                                    new Pose(57, 84)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.9, () -> { spindexer.stopIntake(); spindexer.startOuttake(); })
                    .build();

            GateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57, 84),
                                    new Pose(42, 64),

                                    new Pose(20, 64.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(20, 64.5),

                                    new Pose(16, 62)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                    .build();


            ShootGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16, 62),

                                    new Pose(57, 84)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
                    .build();

            MoveToThirdRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57, 84),
                                    new Pose(58.000, 46.000),
                                    new Pose(52.000, 38.000),
                                    new Pose(45.000, 39.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(45.000, 39.000),

                                    new Pose(12.500, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();


            ShootThirdRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.500, 36.000),

                                    new Pose(57, 84)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.8, () -> spindexer.stopIntake())
                    .addParametricCallback(0.9, () -> spindexer.startOuttake())
                    .build();

            IntakeFirstRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57, 84),

                                    new Pose(20.500, 84)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            ShootFirstRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.500, 84),

                                    new Pose(51, 115)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.75, () -> spindexer.stopIntake())
                    .addParametricCallback(0.85, () -> spindexer.startOuttake())
                    .build();
        }
    }


    private Follower  follower;
    private Paths     paths;
    private Timer     pathTimer, opmodeTimer, intakeTimer;
    private int      pathState;

    private Shooter   shooter;
    private Spindexer spindexer;

    private double angle       = 44;
    private double odoDist     = 66;
    private String targetMotif = "Null";

    public static final Pose START_POS = new Pose(31, 135, Math.toRadians(270));

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── PRELOAD ───────────────────────────────────────────────────────

            case 0:
                follower.setMaxPower(0.9);
                follower.followPath(paths.MoveToShootPreload, true);
                spindexer.startIntake();
                setPathState(1);
                break;

            case 1:
                if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(2);
                } else if (!follower.isBusy() && !targetMotif.equals("Null")) {
                    setPathState(2);
                }
                break;

            case 2:
                angle = 50;
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 2.0) {
                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(3);
                }
                break;

            case 3:
                if (spindexer.outtakeStage == -1) {
                    setPathState(10);
                }
                break;

            // ── SECOND ROW ───────────────────────────────────────────────────

            case 10:
                follower.followPath(paths.IntakeSecondRow, true);
                spindexer.startIntake();
                angle = 340;
                odoDist = 64;
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SecondRowToGate, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.GateToShoot, true);
                    setPathState(13);
                }
                break;

            case 13:
                spindexer.stopIntake();
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(20);
                }
                break;

            // ── GATE INTAKE ───────────────────────────────────────────────────

            case 20:
                follower.followPath(paths.GateIntake, true);
                spindexer.startIntake(); // called once here, not in a loop
                angle = 345;
                setPathState(21);
                break;

            // Wait for robot to finish the gate path
            case 21:
                if (!follower.isBusy()) {
                    setPathState(22);
                    intakeTimer.resetTimer();
                }
                break;

            // Stay at gate for 2 seconds (or leave early if full)
            case 22:
                if (spindexer.artifactCount == 3 || intakeTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.ShootGate, true);
                    setPathState(23);
                }
                break;

            // ShootGate callbacks handle stopIntake + startOuttake automatically
            case 23:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    follower.followPath(paths.MoveToThirdRow, true);
                    setPathState(30);
                }
                break;

            // ── THIRD ROW ─────────────────────────────────────────────────────

            case 30:
                spindexer.startIntake();
                angle = 1;
                odoDist = 65;
                if (!follower.isBusy()) {
                    setPathState(31);
                }
                break;

            case 31:
                follower.followPath(paths.ShootThirdRow, true);
                setPathState(32);
                break;

            case 32:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(40);
                }
                break;

            // ── FIRST ROW ─────────────────────────────────────────────────────

            case 40:
                spindexer.startIntake();
                follower.followPath(paths.IntakeFirstRow, true);
                angle = 336;
                odoDist = 15;
                setPathState(41);
                break;

            case 41:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootFirstRow, true);
                    setPathState(42);
                }
                break;

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

    @Override
    public void init() {
        Constant.ALLIANCE = "BLUE";

        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        intakeTimer = new Timer();

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
    public void init_loop() {

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intakeTimer.resetTimer();
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
            shooter.updateTurret(110, 0);
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
        Constant.AUTON_LAST_X           = 113.5 - p.getX();
        Constant.AUTON_LAST_Y           =   8 - p.getY();
        Constant.AUTON_LAST_HEADING_RAD = p.getHeading() - Math.PI;
        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);

        telemetry.addData("Slots",         slotVisual.toString());
        telemetry.addData("Path State",    pathState);
        telemetry.addData("Intake Stage",  spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
        
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("Motif",         targetMotif);
        telemetry.addData("Turret Angle",  angle);
        telemetry.addData("Odo Dist",      odoDist);
        telemetry.addData("Velo Error",    "%.1f",
                shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());
        telemetry.addData("Target Color",  spindexer.targetColor);
        telemetry.addData("Max Power",     follower.getMaxPowerScaling());
        telemetry.addData("Drive Pos",       "X=%.1f  Y=%.1f", p.getX(), p.getY());
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