package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.teamcode.SubSystems.SubFlywheel;
import org.firstinspires.ftc.teamcode.SubSystems.SubData;
import org.firstinspires.ftc.teamcode.SubSystems.SubIntake;

@Autonomous(name = "Drive back and shoot")
public class driveBackShoot extends LinearOpMode {

    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;

    SubFlywheel flywheel = null;
    SubIntake intake = null;

    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    int fRPosition, fLPosition, bRPosition, bLPosition = 0;
    int tolerance = 30;

    public void driveToRelative(int fr, int fl, int br, int bl, double power) {

        frontRight.setTargetPosition(fRPosition - fr);
        frontLeft .setTargetPosition(fLPosition - fl);
        backRight .setTargetPosition(bRPosition - br);
        backLeft  .setTargetPosition(bLPosition - bl);

        fRPosition -= fr;
        fLPosition -= fl;
        bRPosition -= br;
        bLPosition -= bl;

        /*
        fRPosition += fr;
        fLPosition += fl;
        bRPosition += br;
        bLPosition += bl;
        */

        frontRight.setPower(power);
        frontLeft .setPower(power);
        backRight .setPower(power);
        backLeft  .setPower(power);

    }

    public boolean isDriving = true;

    public void waitForActionCompletion(){

        isDriving = true;

        while(opModeIsActive() && isDriving) {

            isDriving =(        (Math.abs(frontLeft.getTargetPosition() - frontLeft.getCurrentPosition()) > tolerance)   &&
                    (Math.abs(frontRight.getTargetPosition() - frontRight.getCurrentPosition()) > tolerance) &&
                    (Math.abs(backLeft.getTargetPosition() - backLeft.getCurrentPosition()) > tolerance)     &&
                    (Math.abs(backRight.getTargetPosition() - backRight.getCurrentPosition()) > tolerance));

            sleep(20);

        }

        sleep(500); // Allow drive motors to come to rest

    }


    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        flywheel = new SubFlywheel(hardwareMap);
        intake = new SubIntake(hardwareMap);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        double offset = SubData.getStartingAngleOffset(SubData.OffsetIDs.RED_CLOSE);
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        flywheel.setFlyWheelRPM(0);

        driveToRelative(-1000, -1000, -1000, -1000, 0.4);
        waitForActionCompletion();

        flywheel.setFlyWheelRPM(6000);
        sleep(3000);

        intake.setLiftPositionWithinRange(1, 0);
        sleep(1500);

        intake.setLiftPositionWithinRange(0,1);
        sleep(1500);

        intake.setLiftPositionWithinRange(0, 0);
        sleep(1500);

        intake.setIntakePower(1);

        intake.setLiftPositionWithinRange(1, 1);
        sleep(1500);

        intake.setLiftPositionWithinRange(0, 0);
        sleep(1500);

        intake.setLiftPositionWithinRange(1, 1);
        sleep(1500);

        driveToRelative(-200, 200, -200, 200, 0.3);
        waitForActionCompletion();
        driveToRelative(200, -200, 200, -200, 0.3);
        waitForActionCompletion();
        driveToRelative(-200, 200, -200, 200, 0.3);
        waitForActionCompletion();
        driveToRelative(200, -200, 200, -200, 0.3);
        waitForActionCompletion();



        // Record final robot heading to preserve it for teleop.
        SubData.setAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - offset);

        telemetry.addData("IMU", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.update();


        sleep(1000);

    }
}