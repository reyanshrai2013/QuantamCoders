package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.Constants;

@Autonomous(name = "BLUE CLOSE SIDE", group = "Pedro Pathing")
public class CloseSideBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Timer launchTimer;
    private Timer intakeTimer;

    private DcMotorEx intake;
    private DcMotorEx launcher;
    private DcMotor feeder;

    private boolean launchStarted = false;
    private boolean drivePowerRamped = false;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        START_LAUNCH_SEQUENCE,
        WAIT_FOR_LAUNCH_SPINUP,
        WAIT_AND_SHOOT,
        DRIVE_SHOOTPOS_INTAKEPOS,
        DRIVE_INTAKEPOSE_SHOOTPOS,
        WAIT_AND_SHOOT1,
        START_LAUNCH_SEQUENCE1,
        WAIT_FOR_LAUNCH_SPINUP1,
        DRIVE_SECOND_INTAKE,
        DRIVE_INTAKEPOS_SHOOTPOS_SECOND,
        START_LAUNCH_SEQUENCE2,
        WAIT_FOR_LAUNCH_SPINUP2,
        WAIT_AND_SHOOT2,
        END_POSE,
        DONE
    }

    PathState pathState;

    // 🔵 MIRRORED POSES (Y = 144 - oldY, heading = -heading)
    private final Pose startPose = new Pose(124.45, 20.40, Math.toRadians(-37));
    private final Pose shootPose = new Pose(103.35, 40.62, Math.toRadians(-39));
    private final Pose shootPoseOne = new Pose(103.35, 40.62, Math.toRadians(-55));
    private final Pose shootPoseTwo = new Pose(103.35, 40.62, Math.toRadians(-60));
    private final Pose startIntakePose = new Pose(96, 58, Math.toRadians(5));
    private final Pose intakePose = new Pose(129.5, 58, Math.toRadians(0));
    private final Pose scndIntakeRdypose = new Pose(96, 80, Math.toRadians(0));
    private final Pose scndIntakePose = new Pose(137.5, 85, Math.toRadians(0));
    private final Pose endPose = new Pose(118.58448275862068,71.4384236453202,Math.toRadians(188));
    private final Pose scndIntakeDonePose = new Pose(112.56157635467981, 84.0591133004926, Math.toRadians(-30));

    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosIntakePos;
    private PathChain driveIntakePoseShootPos;
    private PathChain driveShootPos2ndIntakePos;
    private PathChain scndIntakeDonetoShootpos;
    private PathChain shootPosToEndPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startIntakePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), startIntakePose.getHeading())
                .addPath(new BezierLine(startIntakePose, intakePose))
                .setLinearHeadingInterpolation(startIntakePose.getHeading(), intakePose.getHeading(), 0)
                .build();

        driveIntakePoseShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, shootPose))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shootPoseOne.getHeading())
                .build();

        driveShootPos2ndIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, scndIntakeRdypose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), scndIntakeRdypose.getHeading(), 0)
                .addPath(new BezierLine(scndIntakeRdypose, scndIntakePose))
                .setLinearHeadingInterpolation(scndIntakeRdypose.getHeading(), scndIntakePose.getHeading(), 0)
                .build();

        scndIntakeDonetoShootpos = follower.pathBuilder()
                .addPath(new BezierLine(scndIntakePose, scndIntakeDonePose))
                .setLinearHeadingInterpolation(scndIntakePose.getHeading(), scndIntakeDonePose.getHeading())
                .addPath(new BezierLine(scndIntakeDonePose, shootPoseTwo))
                .setLinearHeadingInterpolation(scndIntakeDonePose.getHeading(), shootPoseTwo.getHeading())
                .build();

        shootPosToEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launch");
        feeder = hardwareMap.get(DcMotor.class, "feed");
        feeder.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(60,0,0,12);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();
        launchTimer = new Timer();
        intakeTimer = new Timer();

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();

        if (!drivePowerRamped && pathTimer.getElapsedTime() > 600) {
            follower.setMaxPower(1.0);
            drivePowerRamped = true;
        }

        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.setMaxPower(0.45);
                drivePowerRamped = false;
                pathTimer.resetTimer();
                follower.followPath(driveStartPosShootPos, true);
                pathState = PathState.START_LAUNCH_SEQUENCE;
                break;

            case START_LAUNCH_SEQUENCE:
                if (!follower.isBusy()) {
                    launcher.setDirection(DcMotorEx.Direction.REVERSE);
                    launcher.setVelocity(1250);
                    launchTimer.resetTimer();
                    pathState = PathState.WAIT_FOR_LAUNCH_SPINUP;
                }
                break;

            case WAIT_FOR_LAUNCH_SPINUP:
                if (launchTimer.getElapsedTime() > 1400) {
                    feeder.setPower(1);
                    intake.setPower(1);
                    pathTimer.resetTimer();
                    launchStarted = true;
                    pathState = PathState.WAIT_AND_SHOOT;
                }
                break;

            case WAIT_AND_SHOOT:
                if (launchStarted && pathTimer.getElapsedTime() > 2500) {
                    feeder.setPower(0);
                    launcher.setVelocity(0);
                    intake.setPower(0);
                    intakeTimer.resetTimer();
                    launchStarted = false;
                    pathState = PathState.DRIVE_SHOOTPOS_INTAKEPOS;
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEPOS:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    drivePowerRamped = false;
                    feeder.setPower(0);
                    intake.setPower(0.8);
                    launcher.setVelocity(-800);
                    if (intakeTimer.getElapsedTime() > 2000) {
                        feeder.setPower(0);
                        launcher.setVelocity(0);
                        intake.setPower(0);
                    }
                    pathTimer.resetTimer();
                    follower.followPath(driveShootPosIntakePos, true);
                    pathState = PathState.DRIVE_INTAKEPOSE_SHOOTPOS;
                }
                break;

            case DRIVE_INTAKEPOSE_SHOOTPOS:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    drivePowerRamped = false;
                    pathTimer.resetTimer();
                    follower.followPath(driveIntakePoseShootPos, true);
                    pathState = PathState.START_LAUNCH_SEQUENCE1;
                }
                break;

            case START_LAUNCH_SEQUENCE1:
                if (!follower.isBusy()) {
                    launcher.setVelocity(1250);
                    launchTimer.resetTimer();
                    pathState = PathState.WAIT_FOR_LAUNCH_SPINUP1;
                }
                break;

            case WAIT_FOR_LAUNCH_SPINUP1:
                if (launchTimer.getElapsedTime() > 1400) {
                    feeder.setPower(1);
                    intake.setPower(1);
                    pathTimer.resetTimer();
                    launchStarted = true;
                    pathState = PathState.WAIT_AND_SHOOT1;
                }
                break;

            case WAIT_AND_SHOOT1:
                if (launchStarted && pathTimer.getElapsedTime() > 2500) {
                    feeder.setPower(0);
                    launcher.setVelocity(0);
                    intake.setPower(0);
                    launchStarted = false;
                    pathState = PathState.DRIVE_SECOND_INTAKE;
                }
                break;

            case DRIVE_SECOND_INTAKE:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    drivePowerRamped = false;
                    feeder.setPower(0);
                    intake.setPower(0.8);
                    launcher.setVelocity(-900);
                    pathTimer.resetTimer();
                    follower.followPath(driveShootPos2ndIntakePos, true);
                    pathState = PathState.DRIVE_INTAKEPOS_SHOOTPOS_SECOND;
                }
                break;

            case DRIVE_INTAKEPOS_SHOOTPOS_SECOND:
                if (!follower.isBusy()) {
                    double currentHeading = follower.getPose().getHeading();
                    scndIntakeDonetoShootpos = follower.pathBuilder()
                            .addPath(new BezierLine(scndIntakePose, scndIntakeDonePose))
                            .setLinearHeadingInterpolation(currentHeading, scndIntakeDonePose.getHeading())
                            .addPath(new BezierLine(scndIntakeDonePose, shootPoseTwo))
                            .setLinearHeadingInterpolation(scndIntakeDonePose.getHeading(), shootPoseTwo.getHeading())
                            .build();
                    follower.setMaxPower(0.5);
                    drivePowerRamped = false;
                    pathTimer.resetTimer();
                    follower.followPath(scndIntakeDonetoShootpos, true);
                    pathState = PathState.START_LAUNCH_SEQUENCE2;
                }
                break;

            case START_LAUNCH_SEQUENCE2:
                if (!follower.isBusy()) {
                    launcher.setVelocity(1250);
                    launchTimer.resetTimer();
                    pathState = PathState.WAIT_FOR_LAUNCH_SPINUP2;
                }
                break;

            case WAIT_FOR_LAUNCH_SPINUP2:
                if (launchTimer.getElapsedTime() > 1600) {
                    feeder.setPower(0.8);
                    intake.setPower(1);
                    pathTimer.resetTimer();
                    launchStarted = true;
                    pathState = PathState.WAIT_AND_SHOOT2;
                }
                break;

            case WAIT_AND_SHOOT2:
                if (launchStarted && pathTimer.getElapsedTime() > 2500) {
                    feeder.setPower(0);
                    launcher.setVelocity(0);
                    intake.setPower(0);
                    launchStarted = false;
                    pathState = PathState.END_POSE;
                }
                break;

            case END_POSE:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    drivePowerRamped = false;
                    pathTimer.resetTimer();
                    follower.followPath(shootPosToEndPos, true);
                    pathState = PathState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }
}
