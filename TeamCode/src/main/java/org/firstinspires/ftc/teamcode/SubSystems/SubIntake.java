package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class SubIntake {

    // Define the min and max values of the lift servos to allow for easier position setting within the predefined range
    private final double LEFT_LIFT_SERVO_MAX  = 0.6;
    private final double LEFT_LIFT_SERVO_MIN  = 0.0;
    private final double RIGHT_LIFT_SERVO_MAX = 1.0;
    private final double RIGHT_LIFT_SERVO_MIN = 0.4;

    private final double LEFT_LIFT_SERVO_RANGE  = Math.abs(LEFT_LIFT_SERVO_MAX  - LEFT_LIFT_SERVO_MIN );
    private final double RIGHT_LIFT_SERVO_RANGE = Math.abs(RIGHT_LIFT_SERVO_MAX - RIGHT_LIFT_SERVO_MIN);

    // Motor and servo definitions
    CRServo    transferWheels;
    DcMotorEx  spaghettiWheelsIntakeMotor;
    Servo      leftLiftServo;
    Servo      rightLiftServo;

    public SubIntake(HardwareMap hardwareMap) {

        // Called from TeleOPMain. Defines hardware.
        spaghettiWheelsIntakeMotor = hardwareMap.get(DcMotorEx.class, "intakeRow");
        spaghettiWheelsIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        spaghettiWheelsIntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        transferWheels = hardwareMap.get(CRServo.class, "transfer");
        transferWheels.setDirection(DcMotorEx.Direction.REVERSE);

        leftLiftServo =  hardwareMap.get(Servo.class, "leftLift" );
        rightLiftServo = hardwareMap.get(Servo.class, "rightLift");
        leftLiftServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setIntakePower(double intakeRowPower) {
        spaghettiWheelsIntakeMotor.setPower(intakeRowPower);
    }

    public void setTransferPower(double transferPow) {

        // TODO: See if still relevant with current robot configuration
        transferWheels.setPower(transferPow);
    }

    public void setLiftPositionWithinRange(double leftServoRelativePosition, double rightServoRelativePosition) {

        // Sets the servo to a relative position within the range of the servo: 0 = minimum, 1 = maximum
        leftLiftServo .setPosition(LEFT_LIFT_SERVO_MIN  + leftServoRelativePosition  * LEFT_LIFT_SERVO_RANGE);
        rightLiftServo.setPosition(RIGHT_LIFT_SERVO_MIN + rightServoRelativePosition * RIGHT_LIFT_SERVO_RANGE);
    }


}
