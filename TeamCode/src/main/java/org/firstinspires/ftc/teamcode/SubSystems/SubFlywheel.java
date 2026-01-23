package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SubFlywheel {

    public DcMotorEx flyWheel;

    final double FLYWHEEL_MAX_RPM = 6000;
    final double FLYWHEEL_MIN_RPM = 0;
    final double FLYWHEEL_TICKS_PER_REVOLUTION = 21; // 6000 rpm motor

    // For setVelocity tuning. TODO: Tune, test if working
    //final double FLYWHEEL_P = 2.5;
    //final double FLYWHEEL_I = 0.1;
    //final double FLYWHEEL_D = 0.2;
    //final double FLYWHEEL_F = 0.5;
    final double FLYWHEEL_P = 0.0035;
    final double FLYWHEEL_I = 0.0;
    final double FLYWHEEL_D = 0.00015;
    final double FLYWHEEL_F = 11.7;


    public SubFlywheel(HardwareMap hardwareMap) {
        flyWheel = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // Allows coasting
        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        flyWheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flywheelPIDF);
        flyWheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // TODO: DcMotor or DcMotorEX?
    }

    public void setFlyWheelRPM(double targetFlywheelRPM) {
        //flyWheel.setVelocity(
        //        Math.max(FLYWHEEL_MIN_RPM,
        //        Math.min(FLYWHEEL_MAX_RPM, targetFlywheelRPM))
        //        * FLYWHEEL_TICKS_PER_REVOLUTION / 60.0 );
        flyWheel.setPower(targetFlywheelRPM / 6000);
    }

}
