package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Your offsets are correct — these do NOT change with board rotation
            .forwardPodY(8)
            .strafePodX(-6.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // Because your Pinpoint is rotated 90° (ports facing forward):
            // The pod plugged into Y is the FORWARD pod
            // The pod plugged into X is the STRAFE pod
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(73.19551482914001)
            .yVelocity(52.29021592027559)
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            ;


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.4)
            .forwardZeroPowerAcceleration(-33.201088434681814)
            .lateralZeroPowerAcceleration(-58.23338115651832)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0.01, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5,0,0.02,0.006))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1,0,0.1,0.6,0.0))
            .centripetalScaling(0.0005)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(1, 70, 1.8, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}