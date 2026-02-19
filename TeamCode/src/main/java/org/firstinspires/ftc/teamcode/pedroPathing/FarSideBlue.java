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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.DualLauncher;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.LimelightAiming;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.Inits;

@Autonomous(name = "BLUE FAR SIDE", group = "Autonomous")
@Configurable
public class FarSideBlue extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private long waitStartTime = 0;

    private long launcherStartTime = 0;
    private boolean waitStarted = false;
    private boolean pathStarted = false;

    private DcMotorEx launcher = null;
    // Dual-launcher support
    private DcMotorEx launcher1 = null;
    private DcMotorEx intake = null;
    private DcMotorEx feed = null;

    // Drive motors for aiming
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // Centralized aiming helper (handles Limelight init internally)
    private LimelightAiming limelightAimer = null;

    // Aiming state
    private boolean aimingStarted = false;
    private boolean aimDone = false;
    private long aimStartTime = 0;

    // Aiming tuning (adjust if needed)
    private final double ROTATION_KP = 0.05;
    private final double AIM_TOLERANCE_DEG = 2.0;
    private final double MIN_ROTATION_POWER = 0.12;
    private final double MAX_ROTATION_POWER = 0.5;
    private final long AIM_TIMEOUT_MS = 2000;

    @Override
    public void init() {
        // Centralized launcher init (dual-launcher helper)
        DualLauncher.init(hardwareMap); // configures launcher motors only
        launcher = DualLauncher.getPrimary();
        launcher1 = DualLauncher.getSecondary();

        // Single-command init for all other motors
        Inits.init(hardwareMap);
        // obtain drive/intake/feed references from Inits
        leftFrontDrive = Inits.getLeftFront();
        rightFrontDrive = Inits.getRightFront();
        leftBackDrive = Inits.getLeftBack();
        rightBackDrive = Inits.getRightBack();
        // prefer DcMotorEx for intake/feed if available
        intake = Inits.getIntakeEx();
        feed = Inits.getFeedEx();
        if (intake == null) { /* keep as null or set via Inits.getIntake() if required */ }
        if (feed == null) { /* keep as null or set via Inits.getFeed() if required */ }

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // create the centralized aimer (single statement enables auto-aim; internal init safe)
        limelightAimer = new LimelightAiming(hardwareMap, ROTATION_KP, AIM_TOLERANCE_DEG, MIN_ROTATION_POWER, MAX_ROTATION_POWER);

        // existing PIDF telemetry and follower setup
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // 🔵 Mirrored starting pose
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
        // Limelight telemetry (use aimer)
        if (limelightAimer.hasValidTarget()) {
            panelsTelemetry.debug("LL_hasTarget", "true");
            panelsTelemetry.debug("LL_tx", String.format("%.2f", limelightAimer.getTxOrNaN()));
            panelsTelemetry.debug("LL_ta", String.format("%.2f", limelightAimer.getTaOrNaN()));
        } else {
            panelsTelemetry.debug("LL_hasTarget", "false");
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
                    // Spin up both launchers to 1500 and start aiming
                    setLauncherVelocity(1500);
                    launcherStartTime = System.currentTimeMillis();
                    aimingStarted = true;
                    aimDone = false;
                    aimStartTime = System.currentTimeMillis();
                    pathStarted = true;
                }
                // handle aiming (non-blocking)
                if (aimingStarted && !aimDone) {
                    Double rot = limelightAimer.getRotationIfNeeded();
                    if (rot == null) {
                        // aligned or no target
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    } else {
                        // keep rotating toward target
                        mecanumDrive(0, 0, rot);
                    }
                    if (!aimDone && System.currentTimeMillis() - aimStartTime >= AIM_TIMEOUT_MS) {
                        // timed out
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    }
                }

                // Start feeding only after spin-up delay AND (aim done or no target)
                if (System.currentTimeMillis() - launcherStartTime >= 1600
                        && (aimDone || !limelightAimer.hasValidTarget())) {
                    feed.setPower(1.0);
                    intake.setPower(1.0);
                }
                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    setLauncherVelocity(0);
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
                    // Spin up both launchers to 1500 and start aiming
                    setLauncherVelocity(1500);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime = System.currentTimeMillis();
                    aimingStarted = true;
                    aimDone = false;
                    aimStartTime = System.currentTimeMillis();
                    pathStarted = true;
                    waitStarted = true;
                }
                if (aimingStarted && !aimDone) {
                    Double rot = limelightAimer.getRotationIfNeeded();
                    if (rot == null) {
                        // aligned or no target
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    } else {
                        // keep rotating toward target
                        mecanumDrive(0, 0, rot);
                    }
                    if (!aimDone && System.currentTimeMillis() - aimStartTime >= AIM_TIMEOUT_MS) {
                        // timed out
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    }
                }

                if (System.currentTimeMillis() - launcherStartTime >= 1600 && feed.getPower() == 0
                        && (aimDone || !limelightAimer.hasValidTarget())) {
                    feed.setPower(1.0);
                    intake.setPower(1);
                }
                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    setLauncherVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 6;
                }
                break;

            case 6:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
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
                    feed.setPower(0);
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
                    // Spin both to 1500 and aim before shooting
                    setLauncherVelocity(1500);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime = System.currentTimeMillis();
                    aimingStarted = true;
                    aimDone = false;
                    aimStartTime = System.currentTimeMillis();
                    pathStarted = true;
                    waitStarted = true;
                }
                if (aimingStarted && !aimDone) {
                    Double rot = limelightAimer.getRotationIfNeeded();
                    if (rot == null) {
                        // aligned or no target
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    } else {
                        // keep rotating toward target
                        mecanumDrive(0, 0, rot);
                    }
                    if (!aimDone && System.currentTimeMillis() - aimStartTime >= AIM_TIMEOUT_MS) {
                        // timed out
                        aimDone = true;
                        aimingStarted = false;
                        mecanumDrive(0, 0, 0);
                    }
                }

                if (System.currentTimeMillis() - launcherStartTime >= 1600 && feed.getPower() == 0
                        && (aimDone || !limelightAimer.hasValidTarget())) {
                    feed.setPower(1.0);
                    intake.setPower(1.0);
                }
                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    setLauncherVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 10;
                }
                break;

            case 10:
                if (!pathStarted) {
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


    // Set both launcher motors (dual support) - delegate to centralized helper
    private void setLauncherVelocity(double velocity) {
        DualLauncher.setLauncherVelocity(velocity); // uses internally-initialized launcher motors
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
        public PathChain Path1, Path3, Path4, Path5, Path7, Path8, Path9, Path11;
        public double Wait2;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.879, 8.412),
                            new Pose(59.6828929, 16.22253129)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(111.5))
                    .build();

            Wait2 = 3750;

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(59.6828929, 16.22253129),
                            new Pose(49.0681502, 35.4492350)))
                    .setLinearHeadingInterpolation(Math.toRadians(111.5), Math.toRadians(183))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(49.0681502, 35.4492350),
                            new Pose(6, 35.64951321)))
                    .setLinearHeadingInterpolation(Math.toRadians(183), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(8.1130737, 35.64951321),
                            new Pose(58.4812239, 18.82614742)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(108.5))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.481, 18.826),
                            new Pose(41.458, 13.717)))
                    .setLinearHeadingInterpolation(Math.toRadians(111.5), Math.toRadians(200))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(41.458, 13.717),
                            new Pose(2.214, 13.717)))
                    .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(200))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(2.214, 13.717),
                            new Pose(59, 19.773)))
                    .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(107))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(59, 19.773),
                            new Pose(59.448275862069, 37.06502463)))
                    .setLinearHeadingInterpolation(Math.toRadians(107), Math.toRadians(200))
                    .build();
        }
    }
}
