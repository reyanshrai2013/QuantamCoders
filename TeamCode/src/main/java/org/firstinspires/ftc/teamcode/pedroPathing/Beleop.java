package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Beleop", group = "StarterBot")
public class Beleop extends OpMode {


    final double LAUNCHER_TARGET_VELOCITY = 1400;
    final double LAUNCHER_MIN_VELOCITY = 500;
    final double LAUNCHER_MAX_VELOCITY = 6000;
    final double LAUNCHER_MIN_ADJUST = 0;
    final double LAUNCHER_STEP = 100;

    final double VELOCITY_TOLERANCE = 75;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotorEx launcherRight = null;

    private DcMotorEx launcherLeft = null;
    private DcMotor intake = null;
    private DcMotor rightFeeder = null;

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

        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launch");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launch1");
        rightFeeder = hardwareMap.get(DcMotor.class, "feed");

        rightFeeder.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        setupLaunchers();

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        rightFeeder.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    private void setupLaunchers() {
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setZeroPowerBehavior(BRAKE);
        launcherRight.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(60, 0, 0, 12)
        );

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setZeroPowerBehavior(BRAKE);
        launcherLeft.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(60, 0, 0, 12)
        );
    }

    @Override
    public void loop() {

        // =================
        // MECANUM DRIVE
        // =================
        mecanumDrive(
                -gamepad2.left_stick_y,
                gamepad2.left_stick_x,
                gamepad2.right_stick_x
        );

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
        boolean rt = gamepad1.right_trigger > 0.1;

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
// dhruvi is a big mac

        if (gamepad1.right_bumper) {
            rightFeeder.setPower(0.55);
        } else if (gamepad1.dpad_down) {
            rightFeeder.setPower(-0.55);
        } else if (gamepad1.dpad_up) {
            rightFeeder.setPower(1);
        }
        else {
            rightFeeder.setPower(0);
        }

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
        telemetry.update();
    }

    private void setLauncherVelocity(double velocity) {
        launcherRight.setVelocity(velocity);
        launcherLeft.setVelocity(velocity);
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
