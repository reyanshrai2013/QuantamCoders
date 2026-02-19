package org.firstinspires.ftc.teamcode.pedroPathing.Utilities;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Centralized dual-launcher utility.
 * Usage:
 *   DualLauncher.init(hardwareMap); // uses default names: "launch","launch1"
 *   DualLauncher.setLauncherVelocity(1500); // uses configured motors
 */
public class DualLauncher {

    // Stored launcher motors when init(...) is used
    private static DcMotorEx primary = null;
    private static DcMotorEx secondary = null;

    // --- Existing API: operate on provided motors ---
    public static void setLauncherVelocity(DcMotorEx primaryMotor, DcMotorEx secondaryMotor, double velocity) {
        if (primaryMotor != null) {
            primaryMotor.setVelocity(velocity);
        }
        if (secondaryMotor != null) {
            secondaryMotor.setVelocity(velocity);
        }
    }

    public static void stop(DcMotorEx primaryMotor, DcMotorEx secondaryMotor) {
        setLauncherVelocity(primaryMotor, secondaryMotor, 0.0);
    }

    // --- New: init helpers that perform provided hardware init snippet but ONLY for launchers ---
    // Default init using expected names
    public static void init(HardwareMap hw) {
        init(hw, "launch", "launch1");
    }

    // Explicit-names init (only launchers)
    public static void init(HardwareMap hw, String launchName, String launch1Name) {
        try {
            primary = hw.get(DcMotorEx.class, launchName);
        } catch (Exception e) {
            primary = null;
        }

        // try to get optional second launcher; allow fallback
        try {
            secondary = hw.get(DcMotorEx.class, launch1Name);
        } catch (Exception e) {
            secondary = null;
        }

        // set directions and PIDF for launchers only; do NOT touch feeders/intakes
        if (primary != null) primary.setDirection(DcMotorEx.Direction.REVERSE);
        if (secondary != null) secondary.setDirection(DcMotorEx.Direction.FORWARD);

        try {
            if (primary != null) {
                primary.setPIDFCoefficients(
                        DcMotorEx.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(60, 0, 0, 12)
                );
            }
            if (secondary != null) {
                secondary.setPIDFCoefficients(
                        DcMotorEx.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(60, 0, 0, 12)
                );
            }
        } catch (Exception ignored) {
            // ignore if controller doesn't allow runtime PIDF set
        }
    }

    // --- New: convenience methods that operate on stored launcher motors ---
    public static void setLauncherVelocity(double velocity) {
        setLauncherVelocity(primary, secondary, velocity);
    }

    public static void stop() {
        stop(primary, secondary);
    }

    public static boolean isInitialized() {
        return primary != null;
    }

    // getters for launcher diagnostics (may return null)
    public static DcMotorEx getPrimary() { return primary; }
    public static DcMotorEx getSecondary() { return secondary; }
}
