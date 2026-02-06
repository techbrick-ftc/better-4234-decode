//todo: finished

package org.firstinspires.ftc.teamcode.autos;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.SubSystems.SubFlywheel;
import org.firstinspires.ftc.teamcode.SubSystems.SubData;
import org.firstinspires.ftc.teamcode.SubSystems.SubIntake;


@Autonomous(name = "States Blue Far")
public class StatesFarBlue extends OpMode {

    private Follower follower;

    SubFlywheel flywheel = null;
    SubIntake intake = null;



    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION_END POSITION
        //DRIVE
        //SHOOT
        STARTTOSHOOT,
        SHOOTPRELOADED,
        DRIVETOFIRSTROW,
        INTAKEFIRSTROW,
        FIRSTINTAKETOSHOOT
    }

    PathState pathState;

    private final Pose startPose = new Pose(22,125,Math.toRadians(143));
    private final Pose shootPose = new Pose(53.5,97, Math.toRadians(138));
    private final Pose firstIntakePose = new Pose(47, 84, Math.toRadians(180));
    private final Pose finalIntakePose = new Pose(16, 84,Math.toRadians(180));


    private PathChain driveStartToShoot, driveToFirstRow, intakeFirstRow, firstRowToShoot;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveToFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstIntakePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstIntakePose.getHeading())
                .build();
        intakeFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(firstIntakePose, finalIntakePose))
                .setLinearHeadingInterpolation(finalIntakePose.getHeading(), finalIntakePose.getHeading())
                .build();
        firstRowToShoot = follower.pathBuilder()
                .addPath(new BezierLine(finalIntakePose, shootPose))
                .setLinearHeadingInterpolation(finalIntakePose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case STARTTOSHOOT:
                follower.followPath(driveStartToShoot, true);
                setPathState(PathState.SHOOTPRELOADED); //resets timer and changes state
                break;
            case SHOOTPRELOADED:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5) {
                    flywheel.setFlyWheelRPM(6000);
                    sleep(2000);
                    intake.liftToPosition(0,1);
                    sleep(1500);
                    intake.liftToPosition(1,0);
                    sleep(1500);
                    intake.liftToPosition(0,0);
                    sleep(1500);
                    intake.setIntakePower(1);
                    sleep(1500);
                    intake.liftToPosition(1,1);
                    sleep(1500);
                    intake.liftToPosition(0,0); //todo: needed?
                    follower.followPath(driveToFirstRow, true);
                    telemetry.addLine("Done Path 1");
                }
                break;
            case DRIVETOFIRSTROW:
                if (!follower.isBusy()) {
                    follower.followPath(intakeFirstRow, true);
                    telemetry.addLine("Done second path");
                }
            case INTAKEFIRSTROW:
                if (!follower.isBusy()) {
                    follower.followPath(firstRowToShoot, true);
                    telemetry.addLine("done third path");
                }
            case FIRSTINTAKETOSHOOT:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 15) {
                    intake.setIntakePower(0);
                    intake.liftToPosition(0,1);
                    sleep(1500);
                    intake.liftToPosition(1,0);
                    sleep(1500);
                    intake.liftToPosition(0,0);
                    intake.setIntakePower(1);
                    sleep(1500);
                    intake.liftToPosition(1,1);
                    sleep(1500);
                    intake.liftToPosition(0,0);
                    telemetry.addLine("done auto");
                }

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        flywheel = new SubFlywheel(hardwareMap);
        intake = new SubIntake(hardwareMap);

        pathState = PathState.STARTTOSHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //todo - add other init for mechaniums?? if needed

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addLine("path state");
        telemetry.addLine(pathState.toString());

    }
}
