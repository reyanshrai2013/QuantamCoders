package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.Constants;

@Autonomous(name = "FAR SIDE BLUE NO SPIKE", group = "No Spike")
@Configurable
public class FarSideBlueNoSpike extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private long waitStartTime = 0;

    private long launcherStartTime = 0;
    private boolean waitStarted = false;
    private boolean pathStarted = false;

    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private DcMotorEx feed = null;

    @Override
    public void init() {

        launcher = hardwareMap.get(DcMotorEx.class, "launch");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        feed = hardwareMap.get(DcMotorEx.class, "feed");

        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        feed.setDirection(DcMotorEx.Direction.REVERSE);

        launcher.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(60, 0, 0, 12)
        );

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // BLUE starting pose
        follower.setStartingPose(new Pose(56.879, 8.412, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (!pathStarted) {
                    follower.followPath(paths.Path1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    waitStartTime = System.currentTimeMillis();
                    waitStarted = true;
                    pathStarted = false;
                    pathState = 1;
                }
                break;

            case 1:
                if (!pathStarted) {
                    launcher.setVelocity(1705);
                    launcherStartTime = System.currentTimeMillis();
                    pathStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 1600 && feed.getPower() == 0) {
                    feed.setPower(0.5);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    launcher.setVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 2;
                }
                break;

            case 2:
                if (!pathStarted) {
                    intake.setPower(0.8);
                    feed.setPower(0.25);
                    launcher.setVelocity(-700);
                    follower.followPath(paths.Path3, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    launcher.setVelocity(0);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted = true;
                    pathStarted = false;
                    pathState = 3;
                }
                break;

            case 3:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0.25);
                    launcher.setVelocity(-700);
                    follower.followPath(paths.Path4, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    launcher.setVelocity(0);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted = true;
                    pathStarted = false;
                    pathState = 4;
                }
                break;

            case 4:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0.25);
                    launcher.setVelocity(-700);
                    follower.followPath(paths.Path5, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    launcher.setVelocity(0);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted = true;
                    pathStarted = false;
                    pathState = 5;
                }
                break;

            case 5:
                if (!pathStarted) {
                    launcher.setVelocity(1705);
                    launcherStartTime = System.currentTimeMillis();
                    pathStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 1600 && feed.getPower() == 0) {
                    feed.setPower(0.5);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    launcher.setVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 6;
                }
                break;

            case 6:
                if (!pathStarted) {
                    follower.followPath(paths.Path7, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 7;
                }
                break;
        }
    }

    public static class Paths {

        public PathChain Path1, Path3, Path4, Path5, Path7;
        public double Wait2, Wait6;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.879, 8.412),
                            new Pose(59.6829, 16.2225)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                    .build();

            Wait2 = 3750;

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(58.481, 18.826),
                            new Pose(41.458, 12.217)))
                    .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(210))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(41.458, 12.217),
                            new Pose(6.214, 14.217)))
                    .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(210))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(6.214, 14.217),
                            new Pose(59.0, 19.773)))
                    .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(112))
                    .build();

            Wait6 = 3750;

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(59.0, 19.773),
                            new Pose(54.7586, 37.0650)))
                    .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(200))
                    .build();
        }
    }
}
