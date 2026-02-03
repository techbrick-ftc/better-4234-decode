package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.SubSystems.SubDrive;
import org.firstinspires.ftc.teamcode.SubSystems.SubFlywheel;
import org.firstinspires.ftc.teamcode.SubSystems.SubIntake;
import org.firstinspires.ftc.teamcode.SubSystems.SubAprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystems.SubData;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp(name="Main TeleOP")
public class TeleOPMain extends LinearOpMode {

    VoltageSensor voltageSensor;

    // Define constants
    final double DRIVE_DEFAULT_POWER  = 1;
    final double DRIVE_SLOW_POWER     = 0.4;

    // In radians. Outtake targets are backups to apriltag based targeting. TODO: Targets need to be tested.
    final double RED_INTAKE_TARGET             =  Math.PI * 0.3;
    final double BLUE_INTAKE_TARGET            = -Math.PI * 0.3;
    final double RED_OUTTAKE_TARGET_PRIMITIVE  =  Math.PI * 0.7;
    final double BLUE_OUTTAKE_TARGET_PRIMITIVE = -Math.PI * 0.7;

    final double FLYWHEEL_HIGH_RPM = 5850;
    final double FLYWHEEL_MED_RPM  = 5500;
    final double FLYWHEEL_LOW_RPM  = 5000;
    final double FLYWHEEL_IDLE_RPM = 0; // Should be left at 0 until end-of-match disable is finished.

    // May need to move to SubAprilTag, also need to check if correct
    final int APRILTAG_RED_ID  = 24;
    final int APRILTAG_BLUE_ID = 20;
    final int APRILTAG_GPP_ID  = 21; // Green,  Purple, Purple
    final int APRILTAG_PGP_ID  = 22; // Purple, Green,  Green
    final int APRILTAG_PPG_ID  = 23; // Purple, Purple, Green

    // Define Variables
    boolean driveAndFireAllowed = true;

    boolean slowMode = false;
    public double drivePow = DRIVE_DEFAULT_POWER;

    // public boolean isRedTeam = false; // TODO: See if can be removed since all usages have been replaced with subData.isRedTeam()
    public double colorTagID;

    // public static double intakeAngle = 0.3 * Math.PI // TODO: Replaced by RED_INTAKE_TARGET and BLUE_INTAKE_TARGET. Can be removed if those work.
    // public static double outtakeAngle = 0.7 * Math.PI

    int headingLockState = 0; // 0 = standard, 1 = locked to fixed position, 2 = apriltag based
    static double headingAngle;
    static double headingTargetAngle = 0;
    double aprilTagRotation;
    // double targetAngleMultiplier = -1; // TODO: Remove once RED_INTAKE_TARGET and BLUE_INTAKE_TARGET validated.
    public boolean fieldCentricActive = true;

    double xP;
    double yP;
    double rP;
    double kP = 0.5;

    // Declare subsystems.
    SubDrive             drive                = null;
    SubFlywheel          flywheel             = null;
    SubIntake            intake               = null;
    SubAprilTagDetection subAprilTagDetection = null;
    SubData              subData              = null;


    @Override
    public void runOpMode() throws InterruptedException {

        // Define subsystems
        drive                = new SubDrive(hardwareMap, SubData.getAngle());
        flywheel             = new SubFlywheel(hardwareMap);
        intake               = new SubIntake(hardwareMap);
        subAprilTagDetection = new SubAprilTagDetection(hardwareMap, telemetry);
        subData              = new SubData();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // Re-initialize the imu to account for heading changes during auto
        //drive.setOffset(Math.PI - SubData.getAngle());

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {


            xP = gamepad1.left_stick_x;
            yP = -gamepad1.left_stick_y;
            rP = -gamepad1.right_stick_x;

            /*
            // IN TESTING: More aggressive max values with more control at lower values using exponential scaling
            xP = Math.max(Math.min(( gamepad1.left_stick_x  * Math.abs(gamepad1.left_stick_x))  * 1.4, 1), -1);
            yP = Math.max(Math.min((-gamepad1.left_stick_y  * Math.abs(gamepad1.left_stick_y))  * 1.4, 1), -1);
            rP = Math.max(Math.min((-gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x)), 1), -1);
                    //Math.max(Math.min(-gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x)) * 1.2, 1), -1);
            */
            if (gamepad1.bWasPressed()) {
                slowMode = !slowMode;
                drivePow = !slowMode ? DRIVE_DEFAULT_POWER : DRIVE_SLOW_POWER;
            }

            if (gamepad1.aWasPressed()) {
                headingTargetAngle = subData.isRedTeam() ? RED_INTAKE_TARGET : BLUE_INTAKE_TARGET;
                headingLockState = 1;
            }

            if (gamepad1.xWasPressed()) {
                headingLockState = 2;
            }

            if (gamepad1.y || (headingLockState == 1 && Math.abs(rP) > 0.5)) {

                // The reason for specifying headingLockState == 1 is so that the driver can drive
                // normally until an apriltag is detected, without constantly having to re-enable
                // auto-targeting when it gets automatically de-activated.
                headingLockState = 0;
            }

            if (gamepad1.leftStickButtonWasPressed() && gamepad1.rightStickButtonWasPressed()) {
                drive.recalibrate();
            }

            if (gamepad1.optionsWasPressed() || gamepad2.optionsWasPressed()) {
                subData.toggleTeam();

                if (subData.isRedTeam()) {
                    colorTagID = APRILTAG_RED_ID;
                } else {
                    colorTagID = APRILTAG_BLUE_ID;
                }
            }
            if (driveAndFireAllowed) {
                if (headingLockState == 0) {

                    // Standard drive
                    drive.To(xP, yP, rP, drivePow, fieldCentricActive);
                } else if (headingLockState == 1) {

                    // Heading locked to set position
                    headingAngle = headingTargetAngle - drive.getImu();

                    if (headingAngle >= Math.PI) {
                        headingAngle -= 2 * Math.PI;
                    }
                    if (headingAngle <= -Math.PI) {
                        headingAngle += 2 * Math.PI;
                    }

                    drive.To(xP, yP, headingAngle * kP, drivePow, fieldCentricActive);
                } else if (headingLockState == 2) {

                    // Apriltags
                    aprilTagRotation = subAprilTagDetection.getRotationCorrection(colorTagID);
                    if(!Double.isNaN(aprilTagRotation)){
                        drive.To(xP, yP, aprilTagRotation, drivePow, fieldCentricActive);
                    } else {
                        drive.To(xP, yP, rP, drivePow, fieldCentricActive);
                    }
                }

                if (gamepad1.dpadRightWasPressed()) {
                    flywheel.setFlyWheelRPM(FLYWHEEL_IDLE_RPM);
                } else if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
                    flywheel.setFlyWheelRPM(FLYWHEEL_HIGH_RPM);
                } else if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
                    flywheel.setFlyWheelRPM(FLYWHEEL_MED_RPM);
                } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
                    flywheel.setFlyWheelRPM(FLYWHEEL_LOW_RPM);
                }

                if (gamepad1.left_trigger >= 0.2) {
                    intake.setIntakePower(1);
                    //intake.setTransferPower(1); TODO: Transfer is now powered off of intake motor, can remove this function fully once tested
                } else if (gamepad1.right_trigger >= 0.2) {
                    intake.setIntakePower(-1);
                    //intake.setTransferPower(1);
                } else {
                    intake.setIntakePower(0);
                    //intake.setTransferPower(0);
                }
            } else {

                // Disables all robot functionality. Only for emergencies or at the end of matches as needed.
                drive.To(0,0,0,0,fieldCentricActive);
                flywheel.setFlyWheelRPM(0);
                intake.setIntakePower(0);
                intake.setTransferPower(0);
                intake.liftToPosition(0,0);
                subAprilTagDetection.stop();
            }

            intake.liftToPosition(gamepad1.left_bumper ? 0 : 1, gamepad1.right_bumper ? 0 : 1);

            // Telemetry below this line -----------------------------------------------------------
            telemetry.addData("RPM Measured", flywheel.flyWheel1.getVelocity());
            telemetry.addData("RPM Measured Fl2", flywheel.flyWheel2.getVelocity());
            telemetry.addData("Team", subData.isRedTeam() ? "RED. [Options] to change." : "BLUE. [Options] to change");
            telemetry.addLine();
            telemetry.addData("Slow Mode", slowMode ? "ON. [B] to change." : "OFF. [B] to change.");
            telemetry.addLine();
            telemetry.addData("IMU (radians)", drive.getRawImu());
            telemetry.addData("Target Heading", headingTargetAngle);
            telemetry.addLine();
            telemetry.addData("Batt. Voltage", voltageSensor.getVoltage());

            telemetry.update();
        }
    }
}

// Developed by Team 4234 for the 2025-2026 FTC season DECODE