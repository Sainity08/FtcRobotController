package org.firstinspires.ftc.teamcode.Nero.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Intake;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Shooter;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class Red extends OpMode {
    Drivetrain drive = new Drivetrain();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    Turret turret = new Turret();
    private Follower follower;
    private final Pose startPose = new Pose(72, 72, (Math.toRadians(0)));
    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);
        shooter.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean Intake;

        drive.driveRobotRelative(y, x, turn);

        intake.intake(gamepad1.left_bumper);
        intake.outtake(gamepad1.right_bumper);
        Intake = gamepad1.right_trigger > .3;
        intake.intakeShoot(Intake);

        shooter.Shooter(gamepad1.x);
        shooter.setRPM(gamepad1.dpad_up, gamepad1.dpad_down);


        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("RPM", shooter.targetRPM);
    }
}
