package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.*;

@TeleOp(name = "RED TeleOp")
public class RedTeleOp extends OpMode {

    private MecanumDrive drive;
    private Shooter shooter;
    private Spindexer spindexer;
    private boolean FieldCentric = false;
    private boolean prevRB1, prevY2, prevA = false;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
    }

    @Override
    public void loop() {
        // 1. DRIVE
        drive.pinpoint.update();
        drive.updateRelativePostion(Constant.GOAL_CENTER_X, Constant.RED_GOAL_CENTER_Y);
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, FieldCentric);

        if (gamepad1.a && !prevA) {
            FieldCentric = !FieldCentric;
        }
        if (gamepad1.x) {
            drive.pinpoint.recalibrateIMU();
            drive.pinpoint.resetPosAndIMU();
        }
        prevA = gamepad1.a;

        // 2. VISION & AIMING
        double dist = drive.distanceToGoal();
        double rawTurretAngle = drive.getAngleToGoal();
        shooter.updateShootingParams(dist,24);
        shooter.updateTurret(rawTurretAngle);

        // 3. INTAKE CONTROL
        if (gamepad1.rightBumperWasPressed()) {
            if (spindexer.intakeStage == -1){
                spindexer.setSpindexer(spindexer.getIntakePos(spindexer.getIntakeIndex()));
                spindexer.startIntake();
            }
            else spindexer.stopIntake();
        }

        // 4. OUTTAKE CONTROL
        if (gamepad2.yWasPressed()) {
            if (spindexer.outtakeStage == -1) spindexer.startOuttake();
            else spindexer.stopOuttake();
        }

        // 5. SUBSYSTEM UPDATES
        shooter.runShooter(spindexer.outtakeStage != -1);
        spindexer.update(gamepad2.left_bumper, shooter.isReady());

        // 6. TELEMETRY
        telemetry.addData("intake stage", spindexer.intakeStage);
        telemetry.addData("outtake stage", spindexer.outtakeStage);
        telemetry.addData("Artifact 1", spindexer.artifacts.isEmpty() ? "unknown" : spindexer.artifacts.get(0));
        telemetry.addData("Artifact 2", spindexer.artifacts.size() > 1 ? spindexer.artifacts.get(1) : "unknown");
        telemetry.addData("Artifact 3", spindexer.artifacts.size() > 2 ? spindexer.artifacts.get(2) : "unknown");
//        telemetry.addData("Velo Error", shooter.calculatedTargetVelocity-shooter.leftShooter.getVelocity());
//        telemetry.addData("Target Velo", shooter.calculatedTargetVelocity);
//        telemetry.addData("Current Velo", shooter.leftShooter.getVelocity());
//        telemetry.addData("Distance (ll)", "%.2f", shooter.limelightDistanceInch);
//        telemetry.addData("Distance (odo)", "%.2f", drive.distanceToGoal());
//        telemetry.addData("Xpod", drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
//        telemetry.addData("Ypod", drive.pinpoint.getPosition().getY(DistanceUnit.INCH));
//        telemetry.addData("raw turret angle", rawTurretAngle);
//        telemetry.addData("calculated turret angle", shooter.calculatedTurretPos);
        telemetry.update();
    }
}