package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Gate", group = "01")
public class GateIntake extends OpMode {

    public static class Paths {
        public PathChain MoveToShootPreload;
        public PathChain MoveToGate;
        public PathChain GateIntake;
        public PathChain ShootIntaked;

        public Paths(Follower follower) {

            MoveToShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(32.729, 136.953),
                            new Pose(56, 80)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                    .build();

            GateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56, 80),
                                    new Pose(42.000, 64.000),
                                    new Pose(16, 65.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(16, 65.5),

                                    new Pose(12, 63)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))
                    .build();

            ShootIntaked = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(12, 63),
                            new Pose(33.977, 67.562),
                            new Pose(56, 80)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

        }
    }

    private Follower follower;
    private Paths    paths;
    private Timer    pathTimer;
    private int      pathState;

    public static final Pose START_POS = new Pose(32.729, 136.953, Math.toRadians(-90));

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.MoveToShootPreload, true);
                setPathState(2);
                break;

//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.MoveToGate, true);
//                    setPathState(2);
//                }
//                break;

            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(paths.GateIntake, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(4);
                }
                break;

            case 4:
                // done, hold position
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower  = Constants.createFollower(hardwareMap);
        paths     = new Paths(follower);
        follower.setStartingPose(START_POS);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X",          "%.2f", follower.getPose().getX());
        telemetry.addData("Y",          "%.2f", follower.getPose().getY());
        telemetry.addData("Heading",    "%.2f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}