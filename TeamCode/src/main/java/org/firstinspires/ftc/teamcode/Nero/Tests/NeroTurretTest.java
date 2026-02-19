package org.firstinspires.ftc.teamcode.Nero.Tests;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Shooter;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Turret;
import org.firstinspires.ftc.teamcode.PID.FlywheelPID;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class NeroTurretTest extends OpMode {
    public Servo LeftTurret, RightTurret;
    private boolean upPressed = false;
    private boolean downPressed = false;
    private final float posStep = (1/12);
    private Follower follower;
    Drivetrain drive = new Drivetrain();
    Turret turret = new Turret();
    Shooter shooter = new Shooter();
    private IMU imu;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        turret.init(hardwareMap);
        drive.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public double position_to_heading(double position) {
        return (2 * Math.PI * position) - Math.PI;
    }



    public double heading_to_position(double heading) {
        return (heading + Math.PI) / (2 * Math.PI);
    }


    @Override
    public void loop() {
        follower.update();
        boolean input1 = gamepad1.left_bumper;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        drive.driveRobotRelative(y, x, turn);
        double heading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

//        if (gamepad1.dpad_up && !upPressed) {
//            targetPosition += posStep;
//            upPressed = true;
//        }
//        if (!gamepad1.dpad_up) {
//            upPressed = false;
//        }
//        if (gamepad1.dpad_down && !downPressed) {
//            targetPosition -= posStep;
//            downPressed = true;
//        }
//        if (!gamepad1.dpad_down) {
//            downPressed = false;
//        }


//        double targetX = 144;
//        double targetY = 144;
//
//        double botX = follower.getPose().getX();
//        double botY = follower.getPose().getY();
//        double botHeading = follower.getPose().getHeading();
//
//        double turretX = botX + 3.557842126 * Math.cos(botHeading);
//        double turretY = botY + 3.557842126 * Math.sin(botHeading);
//
//        double targetGlobalHeading = Math.atan2(targetY - turretY, targetX - turretX);
//
//        double turretRelativeHeading = targetGlobalHeading - botHeading;
//
//        turretRelativeHeading = MathFunctions.clamp(turretRelativeHeading, -Math.toRadians(149), Math.toRadians(149));
//
//        double targetPosition = heading_to_position(turretRelativeHeading);
//
//        if (targetPosition>1){
//            targetPosition = targetPosition-1;
//        }
//
//        if (targetPosition<0){
//            targetPosition = 1+targetPosition;
//        }
//        targetPosition = MathFunctions.clamp(targetPosition, 0.0813, 0.916);
//
//        LeftTurret.setPosition(targetPosition);
//        RightTurret.setPosition(targetPosition);
//



//        double turretHeading = position_to_heading((LeftTurret.getPosition() + RightTurret.getPosition())/2);
//        double targetTurretHeading = Math.atan2((targetY-turretY),(targetX-turretX));

//
//        double globalTurretHeading = (botHeading + targetTurretHeading);
//        globalTurretHeading = MathFunctions.clamp(globalTurretHeading, -150,150);
//        double actualPosition = ((globalTurretHeading + 180)/360);
//        actualPosition = MathFunctions.clamp(targetPosition, 0.08333, 0.91667);

//
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getHeading()));
//        telemetry.addData("servo position", targetPosition);
////        telemetry.addData("Actual Turret Heading", turretHeading);
////        telemetry.addData("Turret Angle", globalTurretHeading);
//        telemetry.update();a

        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.redGoalX,turret.redGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(), true, false, false);
        double distance = Shooter.distance2D(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()), turret.redGoalX,turret.redGoalY);
        telemetry.addData("distance", distance);
        telemetry.addData("turret x", turret.turretpositionX(follower.getPose().getX(),follower.getPose().getY(),follower.getPose().getHeading()));
        telemetry.addData("turret y", turret.turretpositionY(follower.getPose().getX(),follower.getPose().getY(),follower.getPose().getHeading()));
        telemetry.addData("turret posF",turret.turretAngle);
        telemetry.addData("turret posB",turret.turretAngle);
        telemetry.addData("Angle",turret.angle);
//        telemetry.addData("RPM", shooter.RPM(distance));
//        telemetry.addData("Hood", shooter.hood(distance));
        telemetry.addData("Distance", distance);



    }
}
