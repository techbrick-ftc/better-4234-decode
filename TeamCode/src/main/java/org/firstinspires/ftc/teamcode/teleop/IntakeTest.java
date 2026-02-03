package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.SubSystems.SubIntake;


@TeleOp(name="intaketest")
public class IntakeTest extends LinearOpMode {


    SubIntake intake = null;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = new SubIntake(hardwareMap);

        CRServo intakeBootRow = hardwareMap.get(CRServo.class, "intakebootrow");



        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            intakeBootRow.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            intake.liftToPosition(gamepad1.left_bumper ? 1:0, gamepad1.right_bumper ? 1:0);








        }


    }


}
