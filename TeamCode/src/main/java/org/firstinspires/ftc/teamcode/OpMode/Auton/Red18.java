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

@Autonomous(name = "\uD83D\uDD34 18 SORTED")
public class Red18 extends OpMode {

    private int cycles_needed = 3;
    private int cycles;
    public class Paths {
        public PathChain MoveToShootPreload;
        public PathChain IntakeSecondRow;
        public PathChain ShootSecondRow;
        public PathChain GateIntake;
        public PathChain ShootGate;
        public PathChain IntakeFirstRow;
        public PathChain ShootFirstRow;

        public Paths(Follower follower) {
            MoveToShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(31.000, 135.000).mirror(),

                                    new Pose(58.789, 77.292).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .addParametricCallback(0.8, () -> spindexer.stopIntake())
                    .addParametricCallback(0.85, () -> spindexer.startOuttake())
                    .build();

            IntakeSecondRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.789, 77.292).mirror(),
                                    new Pose(56.658, 59.540).mirror(),
                                    new Pose(41, 59.5).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(41, 59.5).mirror(),

                                    new Pose(10, 59.5).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ShootSecondRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10, 59.5).mirror(),
                                    new Pose(58.5, 77).mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.8, () -> spindexer.stopIntake())
                    .addParametricCallback(0.9, () -> spindexer.startOuttake())
                    .build();

            GateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.5, 77).mirror(),
                                    new Pose(42.000, 64.000).mirror(),
                                    new Pose(16, 65.5).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(16, 65.5).mirror(),

                                    new Pose(12.5, 61.5).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            ShootGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.5, 61.5).mirror(),

                                    new Pose(58.5, 77).mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
                    .build();

            IntakeFirstRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.565, 77.292).mirror(),
                                    new Pose(48.911, 85.447).mirror(),
                                    new Pose(16, 83.553).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            ShootFirstRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16, 83.553).mirror(),

                                    new Pose(48.764, 113.435).mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.8, () -> spindexer.stopIntake())
                    .addParametricCallback(0.9, () -> spindexer.startOuttake())
                    .build();
        }
    }


    private Follower  follower;
    private Paths     paths;
    private Timer     pathTimer, opmodeTimer, intakeTimer;
    private int      pathState;

    private Shooter   shooter;
    private Spindexer spindexer;

    private double angle       = 130;
    private double odoDist     = 70;
    private String targetMotif = "PPP";

    public final Pose START_POS = new Pose(31, 135, Math.toRadians(270)).mirror();

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── PRELOAD ───────────────────────────────────────────────────────

            case 0:
                follower.setMaxPower(1);
                setPathState(1);
                break;

            case 1:
                follower.followPath(paths.MoveToShootPreload, true);
                spindexer.startIntake();
                setPathState(10);
                break;

            case 2:
                if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(2);
                } else if (!follower.isBusy() && !targetMotif.equals("Null")) {
                    setPathState(2);
                }
            break;

            // SECOND ROW

            case 10:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    angle = 200;
                    follower.followPath(paths.IntakeSecondRow, true);
                    spindexer.startIntake();
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootSecondRow, true);
                    setPathState(20);
                }
                break;

            // GATE INTAKE

            case 20:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    angle = 205;
                    follower.followPath(paths.GateIntake, true);
                    spindexer.startIntake();
                    setPathState(21);
                }
                break;

            // Wait for robot to finish the gate path
            case 21:
                if (!follower.isBusy()) {
                    intakeTimer.resetTimer();
                    setPathState(22);
                }
                break;


            case 22:
                if (spindexer.artifactCount == 3 || intakeTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootGate, true);
                    setPathState(67);
                }
                break;

            // CYCLING
            case 67:
                if (!follower.isBusy() && spindexer.outtakeStage == -1 && cycles < cycles_needed) {
                    cycles += 1;
                    follower.followPath(paths.GateIntake, true);
                    spindexer.startIntake();
                    setPathState(21);
                } else if (!follower.isBusy() && spindexer.outtakeStage == -1 && cycles >= cycles_needed){
                    odoDist = 15;
                    follower.followPath(paths.IntakeFirstRow, true);
                    spindexer.startIntake();
                    setPathState(30);
                }
                break;

            // FIRST ROW

            case 30:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootFirstRow, true);
                    setPathState(31);
                }
                break;

            case 31:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(99);
                }
                break;

            // DONE
            case 99:
                break;
        }
    }

    @Override
    public void init() {
        Constant.ALLIANCE = "RED";

        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        intakeTimer = new Timer();

        shooter   = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        spindexer.setSpindexer(Constant.INTAKE_POS1);
        shooter.setTurretPosition(0.2);

        follower = Constants.createFollower(hardwareMap);
        paths    = new Paths(follower);
        follower.setStartingPose(START_POS);
        spindexer.noSort = false;
        cycles = 1;
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

        shooter.updateShootingParams(odoDist, 20, spindexer.outtakeStage != -1);
        shooter.updateTurret(angle, 0);
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
        telemetry.addData("artifactCount", spindexer.artifactCount);
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