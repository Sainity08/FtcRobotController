package org.firstinspires.ftc.teamcode.Nero.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Intake;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Shooter;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Turret;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.file;
import org.firstinspires.ftc.teamcode.Nero.PID.NeroFlywheelPIDF;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Red Op Mode")
public class RedOpMode extends OpMode {
    Drivetrain drive = new Drivetrain();
    Intake intake = new Intake();
    NeroFlywheelPIDF pid;
    private Servo turretServoFront, turretServoBack;
    Shooter shooter = new Shooter();
    Turret turret = new Turret();
    private Follower follower;
    private Limelight3A limelight;
    private GoBildaPinpointDriver odom;
    file fileManager = new file();

    boolean autoAim = false;
    boolean xWasPressed = false;
    public Servo hood;
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    //Pedro
//    private final Pose startPose = new Pose(72, 72, (Math.toRadians(0)));

    //Roadrunner
//    private final Pose startPose = new Pose(0, 0, (Math.toRadians(0)));
    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);
        shooter.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        fileManager.init();
        fileManager.FileRead();
        Pose startPose = new Pose(fileManager.routine.get(0),fileManager.routine.get(1),fileManager.routine.get(2));
        telemetry.addData("points",fileManager.routine);
        follower.setStartingPose(startPose);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "2");
        hood = hardwareMap.get(Servo.class,"hood");
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean isValid = limelight.getLatestResult().isValid();
        drive.driveRobotRelative(y, x, turn);

        //Intake Commands
        boolean input3 = gamepad1.left_trigger > 0.3;
        boolean input4 = gamepad1.right_trigger > 0.3;
        double distance = Shooter.distance2D(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()), turret.redGoalX,turret.redGoalY);
        intake.intake(gamepad1.left_bumper, gamepad1.right_bumper, input3, distance);

        shooter.newRPM(distance);
        shooter.newHood(distance);
        shooter.ShooterGo(gamepad1.x);
        turret.update(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),Math.toDegrees(follower.getHeading()),turret.blueGoalX,turret.blueGoalY,follower.getVelocity().getXComponent(),follower.getVelocity().getYComponent(), false, false , gamepad1.a);

        if(gamepad1.dpad_left){
            follower.setPose(new Pose (144-134.5,8.5,Math.toRadians(0)));
        }
        if(gamepad1.dpad_right){
            follower.setPose(new Pose (144-22.5,127.5,follower.getHeading()));
        }
        if(gamepad1.dpadDownWasPressed()) {
            turret.offset -= .005;
        }

        if(gamepad1.dpadUpWasPressed()) {
            turret.offset += .005;
        }


        follower.update();
        telemetry.addData("Distance", distance);
        telemetry.addData("RPM", shooter.targetRPM);
        telemetry.addData("Current RPM", shooter.avgRPM);
        telemetry.addData("Hood", shooter.targetHood);
        telemetry.addData("Calculated Hood", shooter.Angle);
        telemetry.addData("Calculated RPM", shooter.RPM);
        telemetry.addLine("-------------------------------------------");
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addLine("-------------------------------------------");
        telemetry.addData("Angular Velocity", turret.omega);
        telemetry.addData("Feedforward", turret.turretFeedForwardServo);
        telemetry.addLine("-------------------------------------------");
        telemetry.addData("Intake power wants to be", intake.intakePower);
        telemetry.addData("Intake power is", intake.actualPower);
        telemetry.addData("can index?", intake.canIntake);
        telemetry.addLine("-------------------------------------------");
        telemetry.addData("turret angle", turret.turretAngle);
        telemetry.addData("turret offset" , turret.offset);
        telemetry.addData("turret degrees" , turret.angle);

    }
}
