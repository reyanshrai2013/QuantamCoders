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

// New imports for dual launcher + drive motors + Limelight
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

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



    // Launcher motors (dual launcher support)
    private DcMotorEx launcher = null;
    private DcMotorEx launcher1 = null;

    private DcMotorEx intake = null;

    private DcMotorEx feed = null;

    // Drive motors (for aiming rotation)
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // Limelight
    private Limelight3A limelight = null;

    // Aiming state
    private boolean aimingStarted = false;
    private boolean aimDone = false;
    private long aimStartTime = 0;

    // Aiming constants (tune if necessary)
    private final double ROTATION_KP = 0.05; // proportional gain
    private final double AIM_TOLERANCE_DEG = 2.0; // degrees
    private final double MIN_ROTATION_POWER = 0.12; // overcome friction
    private final double MAX_ROTATION_POWER = 0.5;
    private final long AIM_TIMEOUT_MS = 2000; // max time to try aiming (ms)



    @Override

    public void init() {

// Initialize motors

        launcher = hardwareMap.get(DcMotorEx.class, "launch");
        // Initialize second launcher
        try {
            launcher1 = hardwareMap.get(DcMotorEx.class, "launch1");
        } catch (Exception e) {
            launcher1 = null; // allow single-motor fallback
        }

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        feed = hardwareMap.get(DcMotorEx.class, "feed");

        // Initialize drive motors for aiming
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");

        // Directions -- follow the same as Constants driveConstants
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Launcher/feeder directions and PIDF (keep existing PIDF since you're already using RUN_USING_ENCODER;
        // we are NOT adding any automatic RPM control here)
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        if (launcher1 != null) {
            launcher1.setDirection(DcMotorEx.Direction.FORWARD); // mirror direction so both spin same physical way
        }
        feed.setDirection(DcMotorEx.Direction.REVERSE);

        launcher.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(60, 0, 0, 12)
        );
        if (launcher1 != null) {
            launcher1.setPIDFCoefficients(
                    DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(60, 0, 0, 12)
            );
        }

        // Limelight initialization (non-blocking polling)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            limelight = null; // continue without Limelight if not present
        }

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

        // Limelight telemetry
        if (limelight != null) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                panelsTelemetry.debug("LL_hasTarget", "true");
                panelsTelemetry.debug("LL_tx", String.format("%.2f", r.getTx()));
                panelsTelemetry.debug("LL_ta", String.format("%.2f", r.getTa()));
            } else {
                panelsTelemetry.debug("LL_hasTarget", "false");
            }
        } else {
            panelsTelemetry.debug("LL", "Not configured");
        }

        panelsTelemetry.debug("AimStatus", aimDone ? "aligned/idle" : (aimingStarted ? "aiming" : "idle"));
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

                    // Spin up both launchers
                    setLauncherVelocity(1500);

                    launcherStartTime = System.currentTimeMillis();

                    // start aiming
                    aimingStarted = true;
                    aimDone = false;
                    aimStartTime = System.currentTimeMillis();

                    pathStarted = true;

                }

                // If aiming is active and not done, run aim step (non-blocking)
                if (aimingStarted && !aimDone) {
                    if (doAimStep()) {
                        aimDone = true;
                        aimingStarted = false;
                        // stop rotation
                        mecanumDrive(0, 0, 0);
                    } else if (System.currentTimeMillis() - aimStartTime >= AIM_TIMEOUT_MS) {
                        // time out
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    }
                }

                // Start feeding only after spin-up time AND (aim is done OR there is no target)
                if (System.currentTimeMillis() - launcherStartTime >= 1600 && feed.getPower() == 0
                        && (aimDone || limelight == null || !hasLimelightTarget())) {

                    feed.setPower(1.0);

                    intake.setPower(1.0);

                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {

                    setLauncherVelocity(0);

                    feed.setPower(0);

                    intake.setPower(0);

                    waitStarted = false;

                    pathStarted = false;

                    // reset aiming flags
                    aimingStarted = false;
                    aimDone = false;

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

                    setLauncherVelocity(-700);

                    follower.followPath(paths.Path4, true);

                    pathStarted = true;

                }

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    feed.setPower(0);

                    setLauncherVelocity(0);

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

                    setLauncherVelocity(1500);

                    launcherStartTime = System.currentTimeMillis();

                    waitStartTime = System.currentTimeMillis();

                    // start aiming
                    aimingStarted = true;
                    aimDone = false;
                    aimStartTime = System.currentTimeMillis();

                    pathStarted = true;

                    waitStarted = true;

                }

                if (aimingStarted && !aimDone) {
                    if (doAimStep()) {
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    } else if (System.currentTimeMillis() - aimStartTime >= AIM_TIMEOUT_MS) {
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    }
                }

                if (System.currentTimeMillis() - launcherStartTime >= 1600 && feed.getPower() == 0
                        && (aimDone || limelight == null || !hasLimelightTarget())) {

                    feed.setPower(1.0);

                    intake.setPower(1);

                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {

                    setLauncherVelocity(0);

                    feed.setPower(0);

                    intake.setPower(0);

                    waitStarted = false;

                    pathStarted = false;

                    aimingStarted = false;
                    aimDone = false;

                    pathState = 6;

                }

                break;



            case 6:

                if (!pathStarted) {

                    intake.setPower(0.8);

                    feed.setPower(0.25);

                    setLauncherVelocity(-700);

                    follower.followPath(paths.Path7, true);

                    pathStarted = true;

                }

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    feed.setPower(0);

                    setLauncherVelocity(0);

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

                    setLauncherVelocity(-700);

                    follower.followPath(paths.Path8, true);

                    pathStarted = true;

                }

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    feed.setPower(0);

                    setLauncherVelocity(0);

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

                    setLauncherVelocity(-700);

                    follower.followPath(paths.Path9, true);

                    pathStarted = true;

                }

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    feed.setPower(0);

                    setLauncherVelocity(0);

                    waitStartTime = System.currentTimeMillis();

                    waitStarted = true;

                    pathStarted = false;

                    pathState = 9;

                }

                break;



            case 9:

                if (!pathStarted) {

                    setLauncherVelocity(1500);

                    launcherStartTime = System.currentTimeMillis();

                    waitStartTime = System.currentTimeMillis();

                    // start aiming
                    aimingStarted = true;
                    aimDone = false;
                    aimStartTime = System.currentTimeMillis();

                    pathStarted = true;

                    waitStarted = true;

                }

                if (aimingStarted && !aimDone) {
                    if (doAimStep()) {
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    } else if (System.currentTimeMillis() - aimStartTime >= AIM_TIMEOUT_MS) {
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    }
                }

                if (System.currentTimeMillis() - launcherStartTime >= 1600 && feed.getPower() == 0
                        && (aimDone || limelight == null || !hasLimelightTarget())) {

                    feed.setPower(1.0);

                    intake.setPower(1.0);

                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {

                    setLauncherVelocity(0);

                    feed.setPower(0);

                    intake.setPower(0);

                    waitStarted = false;

                    pathStarted = false;

                    aimingStarted = false;
                    aimDone = false;

                    pathState = 10;

                }

                break;



            case 10:

                if (!pathStarted) {
                    intake.setPower(1.0);

                    feed.setPower(0);

                    setLauncherVelocity(-700);

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

    // Helper: check Limelight target presence
    private boolean hasLimelightTarget() {
        if (limelight == null) return false;
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid();
    }

    // Perform one aim step; returns true if aligned (within tolerance)
    private boolean doAimStep() {
        if (limelight == null) {
            return true; // no limelight -> treat as aligned so we don't block firing
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return true; // no valid target -> treat as aligned (we'll skip aiming)
        }
        double tx = result.getTx(); // degrees
        double rotationPower = tx * ROTATION_KP;

        if (Math.abs(tx) < AIM_TOLERANCE_DEG) {
            // aligned
            return true;
        } else {
            // ensure minimum command
            if (Math.abs(rotationPower) < MIN_ROTATION_POWER) {
                rotationPower = Math.copySign(MIN_ROTATION_POWER, rotationPower);
            }
            // cap
            rotationPower = Math.max(-MAX_ROTATION_POWER, Math.min(MAX_ROTATION_POWER, rotationPower));
            // Apply rotation only (no forward/strafe)
            mecanumDrive(0, 0, rotationPower);
            return false;
        }
    }

    // Set both launcher motors (dual support)
    private void setLauncherVelocity(double velocity) {
        if (launcher != null) launcher.setVelocity(velocity);
        if (launcher1 != null) launcher1.setVelocity(velocity);
    }

    // Mecanum drive helper (simple power set)
    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double leftFrontPower = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower = (forward - strafe + rotate) / denominator;
        double rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
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
