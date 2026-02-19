package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.DualLauncher;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.Inits;
import org.firstinspires.ftc.teamcode.pedroPathing.Utilities.LimelightAiming;


@TeleOp(name = "Beleop", group = "StarterBot")
public class Beleop extends OpMode {

    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = -1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1400;
    final double LAUNCHER_MIN_VELOCITY = 500;
    final double LAUNCHER_MAX_VELOCITY = 6000;
    final double LAUNCHER_MIN_ADJUST = 0;
    final double LAUNCHER_STEP = 50;

    final double VELOCITY_TOLERANCE = 75;

    // Limelight tracking constants
    final double ROTATION_KP = 0.05; // Proportional gain for rotation (tune this)
    final double TARGET_TOLERANCE = 2.0; // Degrees of acceptable error
    final double MIN_ROTATION_POWER = 0.15; // Minimum power to overcome friction
    final double MAX_ROTATION_POWER = 0.5; // Maximum rotation speed


    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotorEx launcherRight = null;
    private DcMotorEx launcherLeft = null;
    private DcMotor intake = null;
    private DcMotor rightFeeder = null;

    private LimelightAiming limelightAimer = null;

    private enum LaunchState {IDLE, SPIN_UP, LAUNCH}

    private LaunchState launchState = LaunchState.IDLE;

    double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    // === Launcher control ===
    double launcherTargetVelocity = 0;

    // Debounce
    boolean lastLB = false;
    boolean lastLT = false;

    @Override
    public void init() {

        // Centralized launcher init
        DualLauncher.init(hardwareMap);
        launcherRight = DualLauncher.getPrimary();
        launcherLeft = DualLauncher.getSecondary();

        // Single-command init for other motors
        Inits.init(hardwareMap);
        leftFrontDrive = Inits.getLeftFront();
        rightFrontDrive = Inits.getRightFront();
        leftBackDrive = Inits.getLeftBack();
        rightBackDrive = Inits.getRightBack();
        intake = Inits.getIntake();
        rightFeeder = Inits.getFeed();

        // single-statement auto-aim enable
        limelightAimer = new LimelightAiming(hardwareMap, ROTATION_KP, TARGET_TOLERANCE, MIN_ROTATION_POWER, MAX_ROTATION_POWER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // note: DualLauncher.init already configured PIDF and directions for launchers, so no extra setupLaunchers() call needed

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        rightFeeder.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Limelight", "Ready");
    }

    private void setupLaunchers() {
        // setupLaunchers() kept for source-compatibility but launcher init/PIDF is handled in DualLauncher.init()
    }

    @Override
    public void loop() {

        // =================
        // DRIVE CONTROL WITH LIMELIGHT TRACKING
        // =================
        // Hold right bumper on gamepad 2 to enable AprilTag tracking
        if (gamepad2.right_bumper) {
            // Auto-tracking mode (while holding right bumper)
            autoTrackAprilTag();
        } else {
            // Manual drive mode
            mecanumDrive(
                    -gamepad2.left_stick_y,
                    gamepad2.left_stick_x,
                    gamepad2.right_stick_x
            );
        }

        // =================
        // LAUNCHER STATE MACHINE
        // =================
        switch (launchState) {

            case IDLE:
                setLauncherVelocity(0);
                rightFeeder.setPower(0);

                if (gamepad1.y) {
                    launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY;
                    setLauncherVelocity(launcherTargetVelocity);
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                setLauncherVelocity(launcherTargetVelocity);

                if (Math.abs(launcherRight.getVelocity() - launcherTargetVelocity)
                        < VELOCITY_TOLERANCE) {
                    launchState = LaunchState.LAUNCH;
                }

                if (gamepad1.b) {
                    launchState = LaunchState.IDLE;
                }
                break;

            case LAUNCH:
                setLauncherVelocity(launcherTargetVelocity);

                if (gamepad1.b) {
                    rightFeeder.setPower(0);
                    launchState = LaunchState.IDLE;
                }
                break;
        }

        // =================
        // RPM ADJUST (DEBOUNCED)
        // =================
        boolean lb = gamepad1.left_bumper;
        boolean lt = gamepad1.left_trigger > 0.1;

        if (lb && !lastLB) {
            launcherTargetVelocity += LAUNCHER_STEP;
            launcherTargetVelocity = Math.min(
                    launcherTargetVelocity, LAUNCHER_MAX_VELOCITY
            );
            setLauncherVelocity(launcherTargetVelocity);
        }

        if (lt && !lastLT) {
            launcherTargetVelocity -= LAUNCHER_STEP;
            launcherTargetVelocity = Math.max(
                    launcherTargetVelocity, LAUNCHER_MIN_ADJUST
            );
            setLauncherVelocity(launcherTargetVelocity);
        }

        lastLB = lb;
        lastLT = lt;

        // =================
        // FEEDER CONTROL
        // =================
        if (gamepad1.right_bumper) {
            rightFeeder.setPower(0.75);
        } else if (gamepad1.dpad_down) {
            rightFeeder.setPower(-0.55);
        } else if (gamepad1.dpad_up) {
            rightFeeder.setPower(1);
        } else {
            rightFeeder.setPower(0);
        }

        // =================
        // INTAKE CONTROL
        // =================
        if (gamepad1.a) {
            intake.setPower(1.0);
        } else if (gamepad1.x) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        // =================
        // TELEMETRY
        // =================
        telemetry.addData("Launcher State", launchState);
        telemetry.addData("Launcher Target RPM", launcherTargetVelocity);
        telemetry.addData("Launcher Actual RPM", launcherRight.getVelocity());
        telemetry.addData(
                "Launcher At Speed",
                Math.abs(launcherRight.getVelocity() - launcherTargetVelocity)
                        < VELOCITY_TOLERANCE
        );
        telemetry.addData("---", "---");
        telemetry.addData("AprilTag Tracking", gamepad2.right_bumper ? "ACTIVE" : "INACTIVE");
        telemetry.addData("(Hold Right Bumper on GP2)", "to track AprilTag");
        telemetry.update();
    }

    /**
     * Auto-tracks AprilTag and rotates the robot to face it
     */
    private void autoTrackAprilTag() {
        // Use centralized aimer to compute rotation (returns null if aligned or no target)
        Double rotationPower = limelightAimer.getRotationIfNeeded();
        if (rotationPower == null) {
            // no target or aligned -> allow manual rotation
            mecanumDrive(
                    -gamepad2.left_stick_y,
                    gamepad2.left_stick_x,
                    gamepad2.right_stick_x
            );
            telemetry.addData("Target Status", limelightAimer.hasValidTarget() ? "LOCKED ON!" : "NO TARGET DETECTED");
        } else {
            mecanumDrive(
                    -gamepad2.left_stick_y,
                    gamepad2.left_stick_x,
                    rotationPower
            );
            telemetry.addData("Target Status", "Tracking...");
            telemetry.addData("Rotation Power", "%.2f", rotationPower);
            telemetry.addData("Target X Offset", "%.2f degrees", limelightAimer.getTxOrNaN());
        }
    }

    private void setLauncherVelocity(double velocity) {
        DualLauncher.setLauncherVelocity(velocity);
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1
        );

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
