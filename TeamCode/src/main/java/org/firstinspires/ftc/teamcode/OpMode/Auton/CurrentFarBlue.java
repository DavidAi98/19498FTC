//package org.firstinspires.ftc.teamcode.OpMode.Auton;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "BLUE Far (cycle)", group = "01")
//public class CurrentFarBlue extends OpMode {
//
//    // =========================================================================
//    //  PATHS
//    // =========================================================================
//
//    public static class Paths {
//        public PathChain ShootPreload;
//        public PathChain MoveToThirdRow;
//        public PathChain IntakeThirdRow;
//        public PathChain ShootThirdRow;
//        public PathChain CycleFarIntake1;
//        public PathChain CycleFarIntake2;
//        public PathChain CycleFarIntake3;
//
//        public Paths(Follower follower) {
//            ShootPreload = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(42.000, 7.500),
//                            new Pose(42.000, 7.500)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            MoveToThirdRow = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(42.000, 7.500),
//                            new Pose(42.053, 35.651)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            IntakeThirdRow = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(42.053, 35.651),
//                            new Pose(9.019, 35.523)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            ShootThirdRow = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(9.019, 35.523),
//                            new Pose(59.506, 10.789)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .setReversed()
//                    .build();
//
//            // Drive from shooting spot to far intake zone
//            CycleFarIntake1 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(59.506, 10.789),
//                            new Pose(12.402, 8.041)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            // Short reposition before the sweep
//            CycleFarIntake2 = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(12.402, 8.041),
//                            new Pose(26.840, 16.782)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            // Bezier sweep arc to collect far-field balls
//            CycleFarIntake3 = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(26.840, 16.782),
//                            new Pose(23.217, 27.357),
//                            new Pose(10.722, 15.370)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(200))
//                    .build();
//        }
//    }
//
//    // =========================================================================
//    //  FIELDS
//    // =========================================================================
//
//    private Follower follower;
//    private Paths    paths;
//    private Timer    pathTimer;
//    private int      pathState;
//
//    public static final Pose START_POS = new Pose(42.000, 7.500, Math.toRadians(180));
//
//    // =========================================================================
//    //  STATE MACHINE
//    // =========================================================================
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//
//            // -----------------------------------------------------------------
//            //  PRELOAD + THIRD ROW  (runs once at start)
//            // -----------------------------------------------------------------
//
//            // Skip zero-length ShootPreload, go straight to row 3
//            case 0:
//                follower.followPath(paths.MoveToThirdRow, true);
//                setPathState(1);
//                break;
//
//            // Sweep across row 3
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.IntakeThirdRow, true);
//                    setPathState(2);
//                }
//                break;
//
//            // Return to shooting position
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.ShootThirdRow, true);
//                    setPathState(3);
//                }
//                break;
//
//            // -----------------------------------------------------------------
//            //  FAR CYCLE LOOP  (repeats for the rest of auto)
//            // -----------------------------------------------------------------
//
//            // Drive to far intake zone
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.CycleFarIntake1, true);
//                    setPathState(4);
//                }
//                break;
//
//            // Short reposition
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.CycleFarIntake2, true);
//                    setPathState(5);
//                }
//                break;
//
//            // Bezier sweep arc
//            case 5:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.CycleFarIntake3, true);
//                    setPathState(6);
//                }
//                break;
//
//            // Return to shoot, then loop back to CycleFarIntake1
//            case 6:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.ShootThirdRow, true);
//                    setPathState(3); // ← loops forever
//                }
//                break;
//        }
//    }
//
//    // =========================================================================
//    //  LIFECYCLE
//    // =========================================================================
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        follower  = Constants.createFollower(hardwareMap);
//        paths     = new Paths(follower);
//        follower.setStartingPose(START_POS);
//    }
//
//    @Override
//    public void init_loop() {}
//
//    @Override
//    public void start() {
//        setPathState(0);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("X",          "%.2f", follower.getPose().getX());
//        telemetry.addData("Y",          "%.2f", follower.getPose().getY());
//        telemetry.addData("Heading",    "%.2f", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {}
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}