package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.Artifact;
import org.firstinspires.ftc.teamcode.TeleOp.Constant;
import org.firstinspires.ftc.teamcode.TeleOp.PIDCoefficients;
import org.firstinspires.ftc.teamcode.TeleOp.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Autonomous(name = "farBlue")
@Config
public class AutonFarBlue extends OpMode {
    private Follower follower;
    private PathChain  firstPickup1, firstPickup2, secondPickup1, secondPickup2,  openRedGate2, firstPickupBreak1, firstPickupBreak2, endPath, secondPickupBreak1, secondPickupBreak2, pickup2ToGate,thirdPickup1,thirdPickup2;
    private Path preLoadShot;
    public static Pose initPos = new Pose(144-80.028, 6.807, Math.toRadians(180));
    public static Pose shootingPos = new Pose(144-86.917, 17.702, Math.toRadians(180));
    public static Pose redGate = new Pose(144-131.475-6, 66.7, Math.toRadians(180));
    public static Pose pickup1Pos = new Pose(144-132.7, 35, Math.toRadians(180));
    public static Pose pickup2Pos = new Pose(144-132.7, 58, Math.toRadians(180));
    public static Pose pickup3Pos = new Pose(144-125, 79,Math.toRadians(180));
    public static Pose endPos1 = new Pose(144-87.409, 27.298, Math.toRadians(180));

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, actionState;

    private DcMotorEx leftShooter, rightShooter;
    private VoltageSensor voltageSensor;


    private double targetVelocity= 2200;
    private double targetHood = 0;
    private double distance = 0;
    private double intakePower = 0;
    private boolean shootComplete;
    private boolean shootOff;

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private DcMotor intake;
    private Servo spindexer1, spindexer2, leftPivot, rightPivot, hood, turret1, turret2;

    private boolean intakeOn = false;
    private int intakeIndex = 1;
    private int outtakeIndex = 1;
    private ArrayList<Artifact> artifacts = new ArrayList<>();
    private boolean currentSet = false;
    private boolean timeReset = false;
    private ElapsedTime time = new ElapsedTime();
    private double currentPosition = Constant.SPINDEXER_INTAKE_POS3;

    private String targetColor = "",targetC="";

    private double nearestPosition = Double.MAX_VALUE;
    private int nearestIndex = -1;

    private double turretServoPosition;
    private double turretHeading;

    double range = Constant.TURRET_RIGHT_MAX-Constant.TURRET_LEFT_MAX;
    private Limelight3A limelight;
    private boolean motifDetected = false,motifCompeted=false,goalDetected=false;
    private VoltageSensor batteryVoltageSensor;

    // --- Control Objects ---
    private PIDFController shooterPID;

    private double filteredAprilX;
    public double aprilx;
    double turretRange = Constant.TURRET_RIGHT_MAX - Constant.TURRET_LEFT_MAX;
    double normalizedHeading;
    int i = 0;



    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

//        if(motifDetected&&motifCompeted)


        if(motifDetected)
            turretControl();
        if (!shootOff) {
            applyShooterPower(targetVelocity);

        }

        if(intakeOn){
            intake.setPower(Constant.INTAKE_POWER);
        }else{
            intake.setPower(0);
        }




        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("turret", turret1.getPosition());
        telemetry.addData("Spin",spindexer1.getPosition());
        telemetry.addData("V", leftShooter.getVelocity());
        telemetry.addData("motif", targetColor);
        telemetry.addData("order", targetC);





        telemetry.update();

    }


    public void buildPaths() {


        preLoadShot = new Path(new BezierCurve(initPos, shootingPos));
        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(), shootingPos.getHeading());


        firstPickupBreak1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        redGate,
                        new Pose(144-64.840, 73.591),
                        new Pose(144-62.652, 54.895)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .build();

        firstPickupBreak2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144-62.652, 54.895),   // from (81.348,54.895)
                        new Pose(144-65.436, 33.017),   // from (78.564,33.017)
                        pickup1Pos))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .build();

        firstPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(144-91.719, 34.6),

                        new Pose(144-91.198, 30.6),

                        pickup1Pos))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
                .build();

        firstPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup1Pos,
                        new Pose(144-113.552, 60), // from (80.155,42.166)
                        shootingPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
                .setLinearHeadingInterpolation(pickup1Pos.getHeading(), shootingPos.getHeading())
                .build();


        pickup2ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pos,
                        new Pose(144-96.93, 55.6),
                        redGate
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
                .build();



        openRedGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(redGate,
                        new Pose(144-96.365, 60.097),
                        shootingPos
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
                .build();

        secondPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(144-88.11, 56),
                        new Pose(144-83.5, 56),
                        pickup2Pos))
                .setLinearHeadingInterpolation(shootingPos.getHeading(),pickup2Pos.getHeading())
                .build();
        secondPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup2Pos,
                        new Pose(144-96, 57),
                        shootingPos))
                .setLinearHeadingInterpolation(shootingPos.getHeading(),pickup2Pos.getHeading())
                .build();





        thirdPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,

                        new Pose(144-90.7,74),
                        new Pose(144-94,81),

                        pickup3Pos))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
                .build();


        thirdPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup3Pos,
                        new Pose(144-96, 55),
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
                shootOff = false;
                setPathState(1);

                break;

            case 1:

                detectMotif();
                if (!follower.isBusy()&&motifDetected&&pathTimer.getElapsedTimeSeconds()>2) {//
                    setActionState(0);
                    setPathState(2);

                }

                break;

            case 2:


                cbaShoot();

                if (shootOff) {
                    setActionState(0);
                    follower.followPath(secondPickup1, true);
                    setPathState(3);

                }
                break;

            case 3://

                if(pathTimer.getElapsedTimeSeconds()>0.7){
                    follower.setMaxPower(0.5);
                    intake();

                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
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
                    shootOff = false;
                    setPathState(6);

                }
                break;

            case 6:

                autoShoot(2);
                if (shootOff) {
                    follower.followPath(firstPickup1, true);
                    setActionState(0);

                    setPathState(7);
                }

                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>0.7){
                    follower.setMaxPower(0.48);
                    intake();

                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(firstPickup2, true);
//                    setActionState(0);
                    setPathState(8);
                }
                break;

            case 8:

                if (!follower.isBusy()) {
                    shootOff = false;

                    setActionState(0);
                    setPathState(9);
                }
                break;
            case 9:

                autoShoot(1);
                if (shootOff) {
                    follower.followPath(endPath, true);
                    setActionState(0);

                    setPathState(13);
                }
                break;
            case 10:
                intake();
                if (!follower.isBusy()) {
                    follower.followPath(thirdPickup2, true);

                    setActionState(0);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    shootOff = false;

                    setActionState(0);
                    setPathState(12);
                }
                break;
            case 12:
                autoShoot(3);
                if (shootOff) {
                    turret1.setPosition(Constant.TURRET_RIGHT_MAX-0.25*turretRange);
                    follower.followPath(endPath, true);

                    setPathState(13);

                }
                break;
            case 13:
                break;
        }
    }



    public void pivotControl(Timer actionTimer) {


        if (actionTimer.getElapsedTimeSeconds() < 0.35) {
            shootComplete = false;
            leftPivot.setPosition(Constant.LEFT_PIVOT_UP);
            rightPivot.setPosition(Constant.RIGHT_PIVOT_UP);
        } else {
            leftPivot.setPosition(Constant.PIVOT_DOWN);
            rightPivot.setPosition(Constant.PIVOT_DOWN);
        }
        if (actionTimer.getElapsedTimeSeconds() > 0.65) {

            shootComplete = true;

            if(!artifacts.isEmpty()){
                artifacts.remove(outtakeIndex-1);
                outtakeIndex=1;
            }



        }
    }




    public void intake() {
        intakeOn = true;

        switch (actionState) {
            case 0:
                i=0;
                setActionState(99);
                break;
            case 99: // Detection
                if(actionTimer.getElapsedTimeSeconds()>0.25)
                    setActionState(1);
                break;
            case 1: // Indexing
                intakeIndex = (intakeIndex % 3) + 1;
                double nPos = (intakeIndex == 1) ? Constant.SPINDEXER_INTAKE_POS1 :
                        (intakeIndex == 2) ? Constant.SPINDEXER_INTAKE_POS2 : Constant.SPINDEXER_INTAKE_POS3;
                spindexer1.setPosition(nPos);
                spindexer2.setPosition(nPos);

                setActionState(2);
                break;
            case 2:

                if(i>=3&&actionTimer.getElapsedTimeSeconds() >= 0.5){
                    setActionState(-1);
                }

                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    i++;
                    setActionState(99);
                }
                break;
            case -1:

                break;
        }

    }





    public void detectMotif(){

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> aprils = result.getFiducialResults();


        if(!aprils.isEmpty()&&!motifDetected){
            for(LLResultTypes.FiducialResult april: aprils)
                switch (april.getFiducialId()) {
                    case 21:
                        targetColor = "GPP";
                        motifDetected = true;
                        break;
                    case 22:
                        targetColor = "PGP";
                        motifDetected = true;
                        break;
                    case 23:
                        targetColor = "PPG";
                        motifDetected = true;
                        break;
                }



        }

    }











    public void autoShoot() {


        if(!artifacts.isEmpty()){
            switch(actionState){
                case 0:

                    targetC += targetColor;
                    setActionState(1);

                    break;

                case 1:


                    if(artifacts.get(outtakeIndex-1).getColor().equals(targetC.substring(0,1))&& Math.abs(nearestPosition - currentPosition)
                            > Math.abs(artifacts.get(outtakeIndex - 1).getPosition() - currentPosition)) {
                        nearestIndex = outtakeIndex;
                        nearestPosition = artifacts.get(outtakeIndex - 1).getPosition();

                        actionTimer.resetTimer();
                        spindexer1.setPosition(nearestPosition);
                        spindexer2.setPosition(nearestPosition);


                        targetC = targetColor.substring(1);
                    }else{
                        outtakeIndex++;
                    }
                    if(outtakeIndex>artifacts.size()){


                        targetC = targetColor.substring(1);
                        setActionState(2);
                    } else if (shootError()&&actionTimer.getElapsedTimeSeconds()>0.6) {
                        setActionState(2);
                    }



                    break;
                case 2:
                    pivotControl(actionTimer);
                    if (shootComplete) {
                        currentPosition = nearestPosition;
                        setActionState(1);
                    }
                    break;

            }
        }else{

            shootOff = true;
        }


    }
    public void bacShoot(){//3-ppg 1-pgp 2-gpp
        switch (actionState) {

            case 0:
                if (actionTimer.getElapsedTimeSeconds()>0.8) {
                    setActionState(1);
                }
                break;
            case 1:
                spindexer1.setPosition(Constant.SPINDEXER_OUTTAKE_POS2);
                spindexer2.setPosition(Constant.SPINDEXER_OUTTAKE_POS2);
                if (actionTimer.getElapsedTimeSeconds()>0.6) {
                    setActionState(2);
                }
                break;
            case 2:
                pivotControl(actionTimer);
                if (shootComplete) {
                    setActionState(3);
                }
                break;
            case 3:
                spindexer1.setPosition(Constant.SPINDEXER_OUTTAKE_POS1);
                spindexer2.setPosition(Constant.SPINDEXER_OUTTAKE_POS1);
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    setActionState(4);
                }
                break;
            case 4:
                pivotControl(actionTimer);
                if (shootComplete) {
                    setActionState(5);
                }
                break;
            case 5:
                spindexer1.setPosition(Constant.SPINDEXER_OUTTAKE_POS3);
                spindexer2.setPosition(Constant.SPINDEXER_OUTTAKE_POS3);
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    setActionState(6);
                }
                break;
            case 6:
                pivotControl(actionTimer);
                if (shootComplete) {
                    setActionState(7);
                }
                break;
            case 7:

                shootOff = true;
                break;
        }
    }
    public void acbShoot(){//2-ppg 3-pgp 1-gpp
        switch (actionState) {

            case 0:
                if (actionTimer.getElapsedTimeSeconds()>0.8) {
                    setActionState(1);
                }
                break;
            case 1:
                spindexer1.setPosition(Constant.SPINDEXER_OUTTAKE_POS1);
                spindexer2.setPosition(Constant.SPINDEXER_OUTTAKE_POS1);
                if (actionTimer.getElapsedTimeSeconds()>0.6) {
                    setActionState(2);
                }
                break;
            case 2:
                pivotControl(actionTimer);
                if (shootComplete) {
                    setActionState(3);
                }
                break;
            case 3:
                spindexer1.setPosition(Constant.SPINDEXER_OUTTAKE_POS3);
                spindexer2.setPosition(Constant.SPINDEXER_OUTTAKE_POS3);
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    setActionState(4);
                }
                break;
            case 4:
                pivotControl(actionTimer);
                if (shootComplete) {
                    setActionState(5);
                }
                break;
            case 5:
                spindexer1.setPosition(Constant.SPINDEXER_OUTTAKE_POS2);
                spindexer2.setPosition(Constant.SPINDEXER_OUTTAKE_POS2);
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    setActionState(6);
                }
                break;
            case 6:
                pivotControl(actionTimer);
                if (shootComplete) {
                    setActionState(7);
                }
                break;
            case 7:

                shootOff = true;
                break;
        }
    }
    public void cbaShoot(){//1-ppg  2-pgp 3-gpp
        switch (actionState) {

            case 0:
                if (actionTimer.getElapsedTimeSeconds()>0.8) {
                    setActionState(1);
                }
                break;
            case 1:
                spindexer1.setPosition(Constant.SPINDEXER_OUTTAKE_POS3);
                spindexer2.setPosition(Constant.SPINDEXER_OUTTAKE_POS3);
                if (actionTimer.getElapsedTimeSeconds()>0.6) {
                    setActionState(2);
                }
                break;
            case 2:
                pivotControl(actionTimer);
                if (shootComplete) {
                    setActionState(3);
                }
                break;
            case 3:
                spindexer1.setPosition(Constant.SPINDEXER_OUTTAKE_POS2);
                spindexer2.setPosition(Constant.SPINDEXER_OUTTAKE_POS2);
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    setActionState(4);
                }
                break;
            case 4:
                pivotControl(actionTimer);
                if (shootComplete) {
                    setActionState(5);
                }
                break;
            case 5:
                spindexer1.setPosition(Constant.SPINDEXER_OUTTAKE_POS1);
                spindexer2.setPosition(Constant.SPINDEXER_OUTTAKE_POS1);
                if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                    setActionState(6);
                }
                break;
            case 6:
                pivotControl(actionTimer);
                if (shootComplete) {
                    setActionState(7);
                }
                break;
            case 7:

                shootOff = true;
                break;
        }
    }


    public void turretControl(){


        /*LLResult results = limelight.getLatestResult();

        if (results.isValid()) {
            List<LLResultTypes.FiducialResult> detection = results.getFiducialResults();

            for (LLResultTypes.FiducialResult april : detection) {
                if (april.getFiducialId() == 20) {
                    aprilx = april.getTargetXDegrees() + Constant.magicConstant;
                }
            }
        } else {
            aprilx = 0;
        }

        filteredAprilX += aprilx * 0.1;*/

        double calculatedTurretPos = -0.062 * turretRange + Constant.TURRET_RIGHT_MAX; // maybe add limelight
        calculatedTurretPos = Math.max(Constant.TURRET_LEFT_MAX,
                Math.min(Constant.TURRET_RIGHT_MAX, calculatedTurretPos));
        turret1.setPosition(calculatedTurretPos - Constant.TURRET_ANTIBACKLASH);
        turret2.setPosition(calculatedTurretPos + Constant.TURRET_ANTIBACKLASH);

    }


    public void autoShoot(int line) {
        // 2. 将你的方法包装成动作
        AutonNearRed.ShootAction acb = () -> acbShoot();
        AutonNearRed.ShootAction bac = () -> bacShoot();
        AutonNearRed.ShootAction cba = () -> cbaShoot();

        AutonNearRed.ShootAction[][] shootMatrix = {
                {acb, bac, cba}, // GPP: line1->acb, line2->bac, line3->cba
                {bac, cba, acb}, // PGP: line1->bac, line2->cba, line3->acb
                {cba, acb, bac}  // PPG: line1->cba, line2->acb, line3->bac
        };

        // 4. 将字符串转为矩阵行索引
        int colorRow = -1;
        switch (targetColor) {
            case "GPP": colorRow = 0; break;
            case "PGP": colorRow = 1; break;
            case "PPG": colorRow = 2; break;
        }


        if (colorRow != -1 && line >= 1 && line <= 3) {

            shootMatrix[colorRow][line - 1].execute();
        }
    }


    public boolean shootError(){
        return targetVelocity-leftShooter.getVelocity()<100;
    }


    private void applyShooterPower(double targetVelo) {
        double currentVelo = leftShooter.getVelocity();
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double ff = (Constant.kV * targetVelo) + (targetVelo > 0 ? Constant.kS : 0);
        shooterPID.setTargetPosition(targetVelo);
        double pid = shooterPID.update(currentVelo);
        double power = (pid + ff) * (Constant.NOMINAL_VOLTAGE / currentVoltage);
        double safePower = Math.max(0, Math.min(1.0, power));
        leftShooter.setPower(safePower);
        rightShooter.setPower(safePower);
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        time.reset();

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        spindexer1 = hardwareMap.get(Servo.class, "spindexer1");
        spindexer2 = hardwareMap.get(Servo.class, "spindexer2");


        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        hood = hardwareMap.get(Servo.class, "RightHood");
        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
        rightPivot = hardwareMap.get(Servo.class, "RightPivot");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        shooterPID = new PIDFController(new PIDCoefficients(Constant.kP, Constant.kI, Constant.kD));

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        turret1.setPosition(Constant.TURRET_RIGHT_MAX);
        turret2.setPosition(Constant.TURRET_RIGHT_MAX);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(initPos);

        hood.setPosition(Constant.HOOD_MAX);


        leftPivot.setPosition(Constant.PIVOT_DOWN);
        rightPivot.setPosition(Constant.PIVOT_DOWN);

        spindexer1.setPosition(Constant.SPINDEXER_INIT);
        spindexer2.setPosition(Constant.SPINDEXER_INIT);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);


        limelight.start();



        intakeOn = false;


    }


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

    }

    public void setActionState(int aState) {
        actionState = aState;
        actionTimer.resetTimer();
        time.reset();
        if(actionState==0) shooterPID.reset();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setActionState(0);


    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

}







