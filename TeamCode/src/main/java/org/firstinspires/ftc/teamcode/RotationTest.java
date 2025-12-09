package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Rotation test", group = "Examples")
public class RotationTest extends OpMode {

    public static Pose initPos = new Pose(79,8,Math.toRadians(90));
    public static Pose shootingPos = new Pose(79, 17.105,Math.toRadians(69));
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3,firstShoot;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        firstShoot = follower.pathBuilder()

                .addPath(new BezierPoint(initPos))
                .setConstantHeadingInterpolation(Math.toRadians(90))

                .build();


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(firstShoot,true);

                    setPathState(1);

                break;
            case 1:

                if(!follower.isBusy()){
                    follower.turnTo(Math.toRadians(90));
                    setPathState(2);
                }



                break;
            case 2:

                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(initPos);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}

//Path1 = follower
//        .pathBuilder()
//      .addPath(
//        new BezierLine(new Pose(118.541, 128.486), new Pose(92.685, 108.597))
//        )
//        .setTangentHeadingInterpolation()
//      .setReversed(true)
//      .build();
//
//Path2 = follower
//        .pathBuilder()
//      .addPath(
//        new BezierCurve(
//                new Pose(92.685, 108.597),
//          new Pose(92.287, 56.287),
//          new Pose(93.680, 59.867),
//          new Pose(131.271, 59.470)
//        )
//                )
//                .setTangentHeadingInterpolation()
//      .build();
//
//Path3 = follower
//        .pathBuilder()
//      .addPath(
//        new BezierCurve(
//                new Pose(131.271, 59.470),
//          new Pose(92.287, 59.669),
//          new Pose(93.680, 57.083),
//          new Pose(92.685, 108.597)
//        )
//                )
//                .setTangentHeadingInterpolation()
//      .setReversed(true)
//      .build();
//
//Path4 = follower
//        .pathBuilder()
//      .addPath(
//        new BezierCurve(
//                new Pose(92.685, 108.597),
//          new Pose(94.276, 84.133),
//          new Pose(83.337, 83.138),
//          new Pose(130.077, 83.735)
//        )
//                )
//                .setTangentHeadingInterpolation()
//      .build();
//
//Path5 = follower
//        .pathBuilder()
//      .addPath(
//        new BezierCurve(
//                new Pose(130.077, 83.735),
//          new Pose(83.337, 82.740),
//          new Pose(94.276, 84.133),
//          new Pose(92.685, 108.398)
//        )
//                )
//                .setTangentHeadingInterpolation()
//      .setReversed(true)
//      .build();

