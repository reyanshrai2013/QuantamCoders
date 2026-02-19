package org.firstinspires.ftc.teamcode.pedroPathing.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Centralized hardware initializers for non-launcher motors.
 * Usage:
 *   Inits.init(hardwareMap);
 *   DcMotor lf = Inits.getLeftFront();
 *   DcMotorEx intakeEx = Inits.getIntakeEx(); // may be null if motor only available as DcMotor
 */
public class Inits {

    private static DcMotor leftFront = null;
    private static DcMotor rightFront = null;
    private static DcMotor leftBack = null;
    private static DcMotor rightBack = null;

    private static DcMotorEx intakeEx = null;
    private static DcMotorEx feedEx = null;
    private static DcMotor intake = null;
    private static DcMotor feed = null;

    private static boolean initialized = false;

    // Default names match your codebase; you can call the overload to use other names
    public static void init(HardwareMap hw) {
        init(hw, "lf", "rf", "lb", "rb", "intake", "feed");
    }

    public static void init(HardwareMap hw,
                            String lfName,
                            String rfName,
                            String lbName,
                            String rbName,
                            String intakeName,
                            String feedName) {
        if (initialized) return;

        // Drive motors (DcMotor)
        try {
            leftFront = hw.get(DcMotor.class, lfName);
        } catch (Exception e) { leftFront = null; }
        try {
            rightFront = hw.get(DcMotor.class, rfName);
        } catch (Exception e) { rightFront = null; }
        try {
            leftBack = hw.get(DcMotor.class, lbName);
        } catch (Exception e) { leftBack = null; }
        try {
            rightBack = hw.get(DcMotor.class, rbName);
        } catch (Exception e) { rightBack = null; }

        // Configure drive motor directions/zero behavior (matches existing code expectations)
        if (leftFront != null) {
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }
        if (leftBack != null) {
            leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            leftBack.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }
        if (rightFront != null) {
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }
        if (rightBack != null) {
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }

        // Intake / Feed - try DcMotorEx first, fall back to DcMotor
        try {
            intakeEx = hw.get(DcMotorEx.class, intakeName);
            intake = intakeEx; // DcMotor reference to same motor
        } catch (Exception e) {
            intakeEx = null;
            try { intake = hw.get(DcMotor.class, intakeName); } catch (Exception ex) { intake = null; }
        }

        try {
            feedEx = hw.get(DcMotorEx.class, feedName);
            feed = feedEx;
        } catch (Exception e) {
            feedEx = null;
            try { feed = hw.get(DcMotor.class, feedName); } catch (Exception ex) { feed = null; }
        }

        if (intake != null) intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        if (feed != null) feed.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        initialized = true;
    }

    // Drive getters
    public static DcMotor getLeftFront() { return leftFront; }
    public static DcMotor getRightFront() { return rightFront; }
    public static DcMotor getLeftBack() { return leftBack; }
    public static DcMotor getRightBack() { return rightBack; }

    // Intake/Feed getters (both DcMotorEx and DcMotor variants)
    public static DcMotorEx getIntakeEx() { return intakeEx; }
    public static DcMotorEx getFeedEx() { return feedEx; }
    public static DcMotor getIntake() { return intake; }
    public static DcMotor getFeed() { return feed; }

    public static boolean isInitialized() { return initialized; }
}
