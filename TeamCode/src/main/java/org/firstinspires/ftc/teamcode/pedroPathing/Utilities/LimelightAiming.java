package org.firstinspires.ftc.teamcode.pedroPathing.Utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * Small utility to centralize limelight aiming logic and hardware init.
 * Usage from an OpMode:
 *   LimelightAiming aimer = new LimelightAiming(hardwareMap);
 *   // optionally: new LimelightAiming(hardwareMap, kp, tolDeg, minPow, maxPow);
 *   Double rot = aimer.getRotationIfNeeded();
 *   if (rot == null) -> aligned or no valid target; otherwise rotate with returned power.
 */
public class LimelightAiming {

    private final Limelight3A limelight;

    // tuning defaults (can be overridden)
    private final double kp;
    private final double aimToleranceDeg;
    private final double minRotationPower;
    private final double maxRotationPower;

    // Constructor that performs the full hardware init/start (safe if limelight absent)
    public LimelightAiming(HardwareMap hw) {
        this(hw, 0.05, 2.0, 0.12, 0.5);
    }

    public LimelightAiming(HardwareMap hw,
                           double kp,
                           double aimToleranceDeg,
                           double minRotationPower,
                           double maxRotationPower) {
        this.kp = kp;
        this.aimToleranceDeg = aimToleranceDeg;
        this.minRotationPower = minRotationPower;
        this.maxRotationPower = maxRotationPower;

        Limelight3A ll = null;
        try {
            ll = hw.get(Limelight3A.class, "limelight");
            // configure and start safely
            ll.setPollRateHz(100);
            ll.pipelineSwitch(0);
            ll.start();
        } catch (Exception e) {
            ll = null;
        }
        this.limelight = ll;
    }

    // Returns true if limelight has a valid target
    public boolean hasValidTarget() {
        if (limelight == null) return false;
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid();
    }

    // Returns tx if available; Double.NaN if not available
    public double getTxOrNaN() {
        if (!hasValidTarget()) return Double.NaN;
        LLResult r = limelight.getLatestResult();
        return r.getTx();
    }

    // Returns ta if available; Double.NaN if not available
    public double getTaOrNaN() {
        if (!hasValidTarget()) return Double.NaN;
        LLResult r = limelight.getLatestResult();
        return r.getTa();
    }

    /**
     * Compute rotation power if rotation is required.
     * Returns:
     *   - a Double rotation power (signed) when robot should rotate,
     *   - null when aligned OR when there is no valid target (caller may treat "no target" as aligned).
     */
    public Double getRotationIfNeeded() {
        if (limelight == null) return null;
        LLResult res = limelight.getLatestResult();
        if (res == null || !res.isValid()) return null;
        double tx = res.getTx();
        if (Math.abs(tx) < aimToleranceDeg) {
            return null; // aligned
        }
        double rotationPower = tx * kp;
        if (Math.abs(rotationPower) < minRotationPower) {
            rotationPower = Math.copySign(minRotationPower, rotationPower);
        }
        rotationPower = Math.max(-maxRotationPower, Math.min(maxRotationPower, rotationPower));
        return rotationPower;
    }
}
