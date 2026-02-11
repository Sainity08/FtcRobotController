package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
        public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.07)
                .forwardZeroPowerAcceleration(-46.70042681491788)
                .lateralZeroPowerAcceleration(-74.474478486357)
                .headingPIDFCoefficients(new PIDFCoefficients(1,0,0, 0.03))
                .centripetalScaling(0.0004);

        public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
                .yVelocity(67.48101710522269)
                .xVelocity(51.572849544014524)
            .rightFrontMotorName("leftBack")
            .rightRearMotorName("leftFront")
            .leftRearMotorName("rightFront")
            .leftFrontMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.25)
            .strafePodX(-5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        public static PathConstraints pathConstraints = new PathConstraints(0.90, 100, 0.8, 0.75);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
