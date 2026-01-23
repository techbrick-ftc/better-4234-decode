package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.SubSystems.SubData;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name = "Red Close Drive Forward Test")
public class RedCloseDriveForwardTest extends LinearOpMode {

    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();



    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;

    int flCurrent = 0;
    int frCurrent = 0;
    int blCurrent = 0;
    int brCurrent = 0;

    double pow;

    public void motorSet(int fl, int fr, int bl, int br, double power) {

        frontLeft.setTargetPosition(flCurrent + fl);
        frontRight.setTargetPosition(frCurrent + fr);
        backLeft.setTargetPosition(blCurrent + bl);
        backRight.setTargetPosition(brCurrent + br);

        flCurrent -= fl;
        frCurrent -= fr;
        blCurrent -= bl;
        brCurrent -= br;

        pow = power/10;

        frontLeft.setPower(pow);
        frontRight.setPower(pow);
        backLeft.setPower(pow);
        backRight.setPower(pow);

    }

    public boolean isDriving() {
        return (
            (Math.abs(frontLeft.getTargetPosition() - frontLeft.getCurrentPosition()) < 30)   &&
            (Math.abs(frontRight.getTargetPosition() - frontRight.getCurrentPosition()) < 30) &&
            (Math.abs(backLeft.getTargetPosition() - backLeft.getCurrentPosition()) < 30)     &&
            (Math.abs(backRight.getTargetPosition() - backRight.getCurrentPosition()) < 30));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        /*
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        double offset = SubData.getStartingAngleOffset(SubData.OffsetIDs.RED_CLOSE);
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);





        waitForStart();

        frontLeft.setPower(-.2);
        frontRight.setPower(-.2);
        backLeft.setPower(-.2);
        backRight.setPower(-.2);

        sleep(2000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        SubData.setAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - offset);

    }
}


