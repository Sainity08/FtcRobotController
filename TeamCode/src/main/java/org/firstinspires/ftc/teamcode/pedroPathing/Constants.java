package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;import com.pedropathing.control.FilteredPIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
        public static FollowerConstants followerConstants = new FollowerConstants()
<<<<<<< Updated upstream
            .mass(9.07)
            .forwardZeroPowerAcceleration(-44.647781505359)
            .lateralZeroPowerAcceleration(-53.104777936320865);
=======
            .mass(8.5)
            .forwardZeroPowerAcceleration(-44.6477881505359)
            .lateralZeroPowerAcceleration(-53.104777936320865)
        .translationalPIDFCoefficients(new PIDFCoefficients(
                                               0.03,
                                                0,
                                               0,
                                               0.015))
        .headingPIDFCoefficients(new PIDFCoefficients(
                                               0.8,
                                                0,
                                               0,
                                               0.01
                                                       ));
>>>>>>> Stashed changes
        public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(67.04942934531866)
            .yVelocity(78.5752760519193);
        public static PinpointConstants localizerConstants = new PinpointConstants()
                .forwardPodY(-3.25)
                .strafePodX(-6.375)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
