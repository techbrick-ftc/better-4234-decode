package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.SubSystems.SubDrive;
import org.firstinspires.ftc.teamcode.SubSystems.SubFlywheel;
import org.firstinspires.ftc.teamcode.SubSystems.SubIntake;
import org.firstinspires.ftc.teamcode.SubSystems.SubAprilTagDetection;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name="old")
public class TeleOpOld extends LinearOpMode {

    // Drive Constants
    double kP = 0.4;

    double slowPow = 0.6;
    double hiPow = 1;

    double intakeAngle = 0.3;
    double shootingHomeAngle = 0.7;

    //Chassis
    double State = 1;
    // 0 - Disabled
    // 1 - Driver controlled
        // 1.0 = Standard Driving
    // 2 - Intaking
        // 2.1 - Intaking, heading held
        // 2.2 - Intaking, free rotating
    // 3 - Shooting
        // 3.1 - Shooting, manual aim
        // 3.2 - Shooting, heading held
        // 3.3 - Shooting, aiming using apriltags


    double angle;
    public double teamColor;
    public double targetAngle;

    // Declare Subsystems
    SubDrive drive = null;
    SubFlywheel flywheel = null;
    SubIntake intake = null;
    SubAprilTagDetection subAprilTagDetection = null;


    @Override
    public void runOpMode() throws InterruptedException {


        // Init Subsystems
        drive = new SubDrive(hardwareMap, Math.PI);
        flywheel = new SubFlywheel(hardwareMap);
        intake = new SubIntake(hardwareMap);
        subAprilTagDetection = new SubAprilTagDetection(hardwareMap, telemetry); //TODO: Check variable

        // Drive variables and controls
        double X_Power;
        double Y_Power;
        double Rotation;

        double gatePosition = 0.5;
        double flyWheelSpeed;

        boolean slowMode = false;
        boolean slowModeToggle;
        boolean slowModeToggleLast = false;

        boolean fieldCentric = true;
        boolean fieldCentricToggle;
        boolean fieldCentricToggleLast = false;

        teamColor = -1;


        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {


            // Drive inputs
            X_Power = gamepad1.left_stick_x;
            Y_Power = -gamepad1.left_stick_y;
            Rotation = gamepad1.right_stick_x;

            flyWheelSpeed = gamepad1.right_trigger * 3000;

            if (gamepad1.b) {
                State = 0;
            } else if (gamepad1.x) {
                State = 2.2;
            } else if (gamepad1.a) {
                State = 3.2;
            }


            // Triggers and booleans
            fieldCentricToggle = (gamepad1.left_stick_button && gamepad1.right_stick_button);
            if (fieldCentricToggle && !fieldCentricToggleLast) {
                fieldCentric = !fieldCentric;
                if (fieldCentric) {
                    drive.recalibrate(); // Recalibrates IMU when field centric enabled
                }
            }
            fieldCentricToggleLast = fieldCentricToggle;

            slowModeToggle = (gamepad1.left_bumper);
            if (slowModeToggle && !slowModeToggleLast) {
                slowMode = !slowMode;
            }
            slowModeToggleLast = slowModeToggle;

            if (gamepad2.back && gamepad2.x) {
                teamColor = -1;
            } else if (gamepad2.back && gamepad2.b) {
                teamColor = 1;
            }


            //region Drive
            if (State == 0) {

                drive.To(0, 0, 0, 0, fieldCentric);
                //intake.Set(0,0, gatePosition);

            } else if (Math.floor(State) == 1) {

                drive.To(X_Power, Y_Power, Rotation, slowMode ? 0.7 : 1, fieldCentric);
               // intake.Set(0, 0, gatePosition);

            } else if (Math.floor(State) == 2) {

                if (State == 2.1) {

                    drive.To(X_Power, Y_Power, Rotation, slowMode ? 0.7 : 1, fieldCentric);
                    // intake.Set(1,1,gatePosition);

                } else if (State == 2.2) {

                    // Calculate target angle based on team color
                    targetAngle = Math.PI - teamColor * 0.3 * Math.PI;
                    angle = -drive.getImu() + Math.PI - targetAngle;

                    // Offset for more efficient rotation
                    if (angle >= Math.PI){
                        angle -= 2*Math.PI;
                    }
                    if (angle <= -Math.PI){
                        angle += 2*Math.PI;
                    }

                    drive.To(X_Power, Y_Power, angle * kP, slowMode ? 0.7 : 1, fieldCentric);
                    // intake.Set(1, 1, gatePosition);

                }

            } else if (Math.floor(State) == 3) {

                if (State == 3.1 ) {

                    drive.To(X_Power, Y_Power, Rotation, slowMode ? 0.7 : 1, fieldCentric);
                    // intake.Set(0, 0, gatePosition);

                } else if (State == 3.2) {

                    // Calculate target angle based on team color
                    targetAngle = Math.PI - teamColor * 0.7 * Math.PI; //TODO: Check Direction, might need to reverse term
                    angle = -drive.getImu() + Math.PI - targetAngle;

                    // Offset for more efficient rotation
                    if (angle >= Math.PI){
                        angle -= 2*Math.PI;
                    }
                    if (angle <= -Math.PI){
                        angle += 2*Math.PI;
                    }

                    drive.To(X_Power, Y_Power, angle * kP, slowMode ? 0.7 : 1, fieldCentric);

                    if (subAprilTagDetection.getOffsetX(20) != 0) {

                        State = 3.3; // Aim using apriltags

                    }

                } else if (State == 3.3) {

                    drive.To(X_Power, Y_Power, subAprilTagDetection.getRotationCorrection(20) * kP, slowMode ? 0.7 : 1, fieldCentric);


                }

                flywheel.setFlyWheelRPM(flyWheelSpeed);


            }
            //endregion

            //region Flywheel
            //flywheel.setFlyWheel(3000 * gamepad1.right_trigger, 2);


            // Telemetry
            telemetry.addData("Team (-1 = blue, 1 = red)", teamColor);
            telemetry.addData("Rotation", drive.getRawImu());
            telemetry.addData("Field Centric?", fieldCentric);
            telemetry.addData("Slow Mode", slowMode);

            if (!gamepad1.square) { // Use to suppress updates to tele data
                telemetry.update();
            }



        }

    }

}
