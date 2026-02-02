package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "NearBlue")
public class NearBlue extends OpMode {

    private Follower follower;
    private PathChain firstPickup1, firstPickup2, secondPickup1, secondPickup2, openRedGate2, firstPickupBreak1, firstPickupBreak2, endPath, secondPickupBreak1, secondPickupBreak2, pickup2ToGate, thirdPickup1, thirdPickup2;
    private Path preLoadShot;
    public static Pose initPos = new Pose(63.359, 6.409, Math.toRadians(180));
    public static Pose shootingPos = new Pose(56.796, 22.476, Math.toRadians(180));
    public static Pose redGate = new Pose(11.923, 70.669, Math.toRadians(180));
    public static Pose pickup1Pos = new Pose(7.552, 35.232, Math.toRadians(180));
    public static Pose pickup2Pos = new Pose(10.591, 59.790, Math.toRadians(180));
    public static Pose pickup3Pos = new Pose(10.735, 83.989, Math.toRadians(180));
    public static Pose endPos1 = new Pose(56.619, 36.669, Math.toRadians(180));

    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState,actionState;

    private Shooter shooter;
    private Spindexer spindexer;





    private String targetMotif = "null";









    public void buildPaths() {

        preLoadShot = new Path(new BezierCurve(initPos, shootingPos));
        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(), shootingPos.getHeading());


        firstPickupBreak1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        redGate,
                        new Pose(64.840, 73.591),
                        new Pose(62.652, 54.895)
                ))
                .setLinearHeadingInterpolation(redGate.getHeading(), redGate.getHeading())
                .build();

        firstPickupBreak2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(62.652, 54.895),   // from (81.348,54.895)
                        new Pose(65.436, 33.017),   // from (78.564,33.017)
                        pickup1Pos))
                .setLinearHeadingInterpolation(redGate.getHeading(), pickup1Pos.getHeading())
                .build();

        firstPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(42.776, 32.392),
                        new Pose(55.318, 38.757),
                        pickup1Pos))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
                .build();

        firstPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup1Pos,
                        new Pose(108.552, 51.633), // from (80.155,42.166)
                        shootingPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
                .setLinearHeadingInterpolation(pickup1Pos.getHeading(), shootingPos.getHeading())
                .build();


        pickup2ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pos,
                        new Pose(24.378, 66.423),
                        redGate
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
                .build();


        openRedGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(redGate,
                        new Pose(53.403, 45.746),
                        shootingPos
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
                .build();

        secondPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(45.749, 71.580),
                        new Pose(36.373, 57.647),
                        pickup2Pos))
                .setLinearHeadingInterpolation(shootingPos.getHeading(), pickup2Pos.getHeading(), 0.4)
                .build();


        thirdPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,

                        new Pose(63.547, 94.097),
                        new Pose(35.122, 83.185),
                        new Pose(50.050, 85.544),
                        pickup3Pos))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
                .build();


        thirdPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup3Pos,

                        shootingPos))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
                .build();


        endPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,

                        endPos1
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
                .build();

    }




    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:


                follower.followPath(preLoadShot);
                spindexer.startIntake();
                setActionState(0);
                setPathState(1);

                break;

            case 1:

                targetMotif = shooter.detectMotif();

                if (!follower.isBusy() && !targetMotif.equals("null")) {
                    setActionState(0);
                    spindexer.startOuttake();
                    setPathState(2);
                }

                break;

            case 2:


                if (spindexer.outtakeStage==-1) {
                    setActionState(0);
                    follower.followPath(secondPickup1, true);
                    spindexer.startIntake();
                    setPathState(3);


                }
                break;

            case 3://

                if(pathTimer.getElapsedTimeSeconds()>0.7){
                    follower.setMaxPower(0.5);
                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    spindexer.stopIntake();
                    follower.followPath(pickup2ToGate, true);
                    setPathState(4);

                }

                break;

            case 4:



                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(openRedGate2, true);
                    setActionState(0);
                    setPathState(5);

                }

                break;

            case 5:


                if (!follower.isBusy()) {
                    spindexer.startOuttake();
                    setPathState(6);

                }
                break;

            case 6:

                if (spindexer.outtakeStage==-1) {
                    follower.followPath(firstPickup1, true);
                    setActionState(0);

                    spindexer.startIntake();
                    setPathState(7);
                }

                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>0.7){
                    follower.setMaxPower(0.48);


                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    spindexer.stopIntake();
                    follower.followPath(firstPickup2, true);
//                    setActionState(0);
                    setPathState(8);
                }
                break;

            case 8:

                if (!follower.isBusy()) {
                    spindexer.startOuttake();

                    setActionState(0);
                    setPathState(9);
                }
                break;
            case 9:

                if (spindexer.outtakeStage==-1) {
                    follower.followPath(endPath, true);
                    setActionState(0);

                    setPathState(13);
                }
                break;
            case 10:

                if (!follower.isBusy()) {
                    follower.followPath(thirdPickup2, true);

                    setActionState(0);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    spindexer.startOuttake();

                    setActionState(0);
                    setPathState(12);
                }
                break;
            case 12:

                if (spindexer.outtakeStage==-1) {
//                    turret1.setPosition(Constant.TURRET_RIGHT_MAX-0.25*turretRange);
                    follower.followPath(endPath, true);

                    setPathState(13);

                }
                break;
            case 13:
                break;
        }
    }








    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        if(!targetMotif.equals("null")){
            shooter.setTurretPosition(0.625);
        }
        shooter.runShooter(spindexer.outtakeStage != -1);
        spindexer.update(targetMotif,shooter.isReady());


        autonomousPathUpdate();

        // 5. Visual Slot Logic
        StringBuilder slotVisual = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if (spindexer.slots[i] == null) {
                slotVisual.append("⚪ "); // Empty
            } else if (spindexer.slots[i].getColor().equals("P")) {
                slotVisual.append("\uD83D\uDFE3 "); // Purple
            } else if (spindexer.slots[i].getColor().equals("G")) {
                slotVisual.append("\uD83D\uDFE2 "); // Green
            }
        }

        // 6. TELEMETRY
        telemetry.addData("Spindexer Slots", slotVisual.toString());

        telemetry.addData("Intake Stage", spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);

        telemetry.addData("Velo Error", "%.1f", shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());

        telemetry.addData("target ticks", spindexer.targetTicks);
        telemetry.addData("current ticks", spindexer.currentTicks);
        telemetry.addData("filteredAprilX", shooter.filteredAprilX);

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
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        shooter = new Shooter(hardwareMap);
        shooter.calculatedTargetVelocity = 2200;
        shooter.setTurretPosition(0.5);
        shooter.setHoodPosition(Constant.HOOD_INIT);
        spindexer = new Spindexer(hardwareMap);


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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

    }

    public void setActionState(int aState) {
        actionState = aState;
        actionTimer.resetTimer();

    }


}

