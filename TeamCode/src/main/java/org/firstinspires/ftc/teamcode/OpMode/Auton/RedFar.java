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

@Autonomous(name = "\uD83D\uDD34 Far (cycle)", group = "02")
public class RedFar extends OpMode {

    // =========================================================================
    //  PATHS
    // =========================================================================

    public class Paths {
        public PathChain IntakeThirdRow;
        public PathChain ShootThirdRow;
        public PathChain CycleFarIntake1;
        public PathChain ShootCycle1;

        public Paths(Follower follower) {

            IntakeThirdRow = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(55.000, 8.000).mirror(),

                                    new Pose(55.000, 20).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.000, 20).mirror(),
                                    new Pose(55.000, 35.500).mirror(),
                                    new Pose(44.000, 35.500).mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(44.000, 35.500).mirror(),

                                    new Pose(11.000, 35.500).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ShootThirdRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.000, 35.500).mirror(),

                                    new Pose(57, 15).mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.8, () -> spindexer.stopIntake())
                    .addParametricCallback(0.9, () -> spindexer.startOuttake())
                    .build();

            // Drive from shooting spot to far intake zone
            CycleFarIntake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57, 15).mirror(),
                                    new Pose(20,5).mirror(),
                                    new Pose(14, 10).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .addPath(
                            new BezierLine(
                                    new Pose(14, 10).mirror(),

                                    new Pose(14, 35).mirror()
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(40))
                    .build();

            ShootCycle1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14, 35).mirror(),

                                    new Pose(57, 15).mirror()
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.8, () -> spindexer.stopIntake())
                    .addParametricCallback(0.9, () -> spindexer.startOuttake())
                    .build();

        }
    }

    // =========================================================================
    //  FIELDS
    // =========================================================================

    private Follower follower;
    private Paths    paths;
    private Timer    pathTimer;
    private Timer    opmodeTimer;
    private int      pathState;
    private Shooter shooter;
    private Spindexer spindexer;

    private double angle       = 290;
    private double odoDist     = 148;
    private String targetMotif = "PPP";

    public final Pose START_POS = new Pose(55, 8, Math.toRadians(90)).mirror();

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================

    public void autonomousPathUpdate() {
        switch (pathState) {

            // -----------------------------------------------------------------
            //  PRELOAD + THIRD ROW  (runs once at start)
            // -----------------------------------------------------------------

            // PRELOAD
            case -1:
                follower.setMaxPower(1);
                setPathState(0);
                break;

            case 0:
                spindexer.startIntake();
                setPathState(1);

            // Sweep across row 3
            case 1:
                if (spindexer.artifactCount == 3 && spindexer.intakeStage == -1) {
                    spindexer.startOuttake();
                    setPathState(2);
                }

//                if (!follower.isBusy()) {
//                    follower.followPath(paths.IntakeThirdRow, true);
//                    setPathState(2);
//                }
                break;

            // Return to shooting position
            case 2:
                if (spindexer.artifactCount == 0 && spindexer.outtakeStage == -1) {
                    spindexer.startIntake();
                    follower.followPath(paths.IntakeThirdRow, true);
                    setPathState(3);
                }
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.ShootThirdRow, true);
//                    setPathState(3);
//                }
                break;

            // -----------------------------------------------------------------
            //  FAR CYCLE LOOP  (repeats for the rest of auto)
            // -----------------------------------------------------------------

            // Drive to far intake zone
            case 3:
                if (!follower.isBusy()) {
                    angle = 225;
                    odoDist = 140;
                    follower.followPath(paths.ShootThirdRow, true);
                    setPathState(4);
                }
                break;

            // Short reposition
            case 4:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    spindexer.startIntake();
                    follower.followPath(paths.CycleFarIntake1, true);
                    setPathState(5);
                }
                break;

            // Bezier sweep arc
            case 5:
                if (!follower.isBusy() || spindexer.artifactCount == 3) {
                    angle = 223;
                    odoDist = 140;
                    follower.followPath(paths.ShootCycle1, true);
                    setPathState(4);
                }
                break;


            // Return to shoot, then loop back to CycleFarIntake1
        }
    }

    // =========================================================================
    //  LIFECYCLE
    // =========================================================================

    @Override
    public void init() {
        Constant.ALLIANCE = "RED";

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        shooter   = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        spindexer.setSpindexer(Constant.INTAKE_POS1);
        shooter.setTurretPosition(0.75);

        follower  = Constants.createFollower(hardwareMap);
        paths     = new Paths(follower);
        follower.setStartingPose(START_POS);
        spindexer.noSort = true;
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(-1);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        shooter.updateShootingParams(odoDist, 20, spindexer.outtakeStage != -1);
        shooter.updateTurret(angle);
        shooter.runShooter(spindexer.outtakeStage != -1);
        spindexer.update(targetMotif, shooter.isReady());

        Pose p = follower.getPose();
        Constant.AUTON_LAST_X = p.getX() - 33.5;
        Constant.AUTON_LAST_Y = p.getY() - 11.5;
        Constant.AUTON_LAST_HEADING_RAD = p.getHeading();
        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);

        StringBuilder slotVisual = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if      (spindexer.slots[i] == null)                    slotVisual.append("⚪ ");
            else if (spindexer.slots[i].getColor().equals("P"))     slotVisual.append("\uD83D\uDFE3 ");
            else if (spindexer.slots[i].getColor().equals("G"))     slotVisual.append("\uD83D\uDFE2 ");
        }

        telemetry.addData("Slots",         slotVisual.toString());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Intake Stage",  spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
        telemetry.update();
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}