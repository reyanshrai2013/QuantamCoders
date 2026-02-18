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



import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "RED FAR SIDE", group = "Autonomous")

@Configurable

public class FarSideRed extends OpMode {



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

// Initialize motors

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



// RED-side starting pose (mirrored across X = 72)

        follower.setStartingPose(new Pose(144 - 56.879, 8.412, Math.toRadians(90)));



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

                    follower.followPath(paths.Path3, true);

                    pathStarted = true;

                }

                if (!follower.isBusy()) {

                    pathStarted = false;

                    pathState = 3;

                }

                break;



            case 3:

                if (!pathStarted) {

                    intake.setPower(0.8);

                    feed.setPower(0);

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

                    follower.followPath(paths.Path5, true);

                    pathStarted = true;

                    intake.setPower(1);

                }

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    pathStarted = false;

                    pathState = 5;

                }

                break;



            case 5:

                if (!pathStarted) {

                    launcher.setVelocity(1700);

                    launcherStartTime = System.currentTimeMillis();

                    waitStartTime = System.currentTimeMillis();

                    pathStarted = true;

                    waitStarted = true;

                }



                if (System.currentTimeMillis() - launcherStartTime >= 1600 && feed.getPower() == 0) {

                    feed.setPower(0.5);

                    intake.setPower(1);

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

                    intake.setPower(0.8);

                    feed.setPower(0.25);

                    launcher.setVelocity(-700);

                    follower.followPath(paths.Path7, true);

                    pathStarted = true;

                }

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    feed.setPower(0);

                    launcher.setVelocity(0);

                    waitStartTime = System.currentTimeMillis();

                    waitStarted = true;

                    pathStarted = false;

                    pathState = 7;

                }

                break;



            case 7:

                if (!pathStarted) {

                    intake.setPower(1);

                    feed.setPower(0.25);

                    launcher.setVelocity(-700);

                    follower.followPath(paths.Path8, true);

                    pathStarted = true;

                }

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    feed.setPower(0);

                    launcher.setVelocity(0);

                    waitStartTime = System.currentTimeMillis();

                    waitStarted = true;

                    pathStarted = false;

                    pathState = 8;

                }

                break;



            case 8:

                if (!pathStarted) {

                    intake.setPower(1.0);

                    feed.setPower(0);

                    launcher.setVelocity(-700);

                    follower.followPath(paths.Path9, true);

                    pathStarted = true;

                }

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    feed.setPower(0);

                    launcher.setVelocity(0);

                    waitStartTime = System.currentTimeMillis();

                    waitStarted = true;

                    pathStarted = false;

                    pathState = 9;

                }

                break;



            case 9:

                if (!pathStarted) {

                    launcher.setVelocity(1700);

                    launcherStartTime = System.currentTimeMillis();

                    waitStartTime = System.currentTimeMillis();

                    pathStarted = true;

                    waitStarted = true;

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

                    pathState = 10;

                }

                break;



            case 10:

                if (!pathStarted) {
                    intake.setPower(1.0);

                    feed.setPower(0);

                    launcher.setVelocity(-700);

                    follower.followPath(paths.Path11, true);

                    pathStarted = true;

                }

                if (!follower.isBusy()) {

                    pathStarted = false;

                    pathState = 11;

                }

                break;







        }

    }



    public static class Paths {

        public PathChain Path1, Path3, Path4, Path5, Path7, Path8, Path9, Path11, Path12, Path14;

        public double Wait2, Wait6, Wait10, Wait13;


        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(144 - 56.879, 8.412),

                            new Pose(144 - 59.6828929, 16.22253129)))

                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(61))

                    .build();



            Wait2 = 3750;



            Path3 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(144 - 59.6828929, 16.22253129),

                            new Pose(144 - 49.0681502, 35.4492350)))

                    .setLinearHeadingInterpolation(Math.toRadians(61), Math.toRadians(-3))

                    .build();



            Path4 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(144 - 49.0681502, 35.4492350),

                            new Pose(144 - 8.11307371349096, 35.64951321)))

                    .setLinearHeadingInterpolation(Math.toRadians(-3), Math.toRadians(0))

                    .build();



            Path5 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(144 - 8.11307371349096, 35.64951321),

                            new Pose(144 - 58.4812239, 18.82614742)))

                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))

                    .build();



            Wait6 = 3750;



            Path7 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(144 - 58.481, 18.826),

                            new Pose(144 - 41.458, 12.217)))

                    .setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(-20))

                    .build();



            Path8 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(144 - 41.458, 12.217),

                            new Pose(144 - 10.214, 12.217)))

                    .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(-20))

                    .build();



            Path9 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(144 - 10.214, 12.217),
                            new Pose(144 - 59, 19.773)))

                    .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(83))

                    .build();



            Wait10 = 3750;



            Path11 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(144 - 59, 19.773),

                            new Pose(84.551724137931, 37.06502463054187)))

                    .setLinearHeadingInterpolation(Math.toRadians(88), Math.toRadians(-20))

                    .build();





        }

    }

}