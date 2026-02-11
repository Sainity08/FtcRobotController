package org.firstinspires.ftc.teamcode.Nero.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Intake;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Shooter;
import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Turret;
import org.firstinspires.ftc.teamcode.Nero.PID.NeroFlywheelPIDF;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp
public class Blue extends OpMode {
    Drivetrain drive = new Drivetrain();
    Intake intake = new Intake();
    NeroFlywheelPIDF pid;
    private Servo turretServoFront, turretServoBack;
    Shooter shooter = new Shooter();
    Turret turret = new Turret();
    private Follower follower;
    private Limelight3A limelight;
    private GoBildaPinpointDriver odom;

    boolean autoAim = false;
    boolean xWasPressed = false;
    public Servo hood;
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    private final Pose startPose = new Pose(72, 72, (Math.toRadians(0)));
    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        turretServoFront = hardwareMap.get(Servo.class,"leftT");
        turretServoBack = hardwareMap.get(Servo.class,"rightT");
        shooter.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        odom = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        limelight.pipelineSwitch(2);
        limelight.start();
        odom.recalibrateIMU();
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
        double tx = limelight.getLatestResult().getTx();
        double ty = limelight.getLatestResult().getTy();
        boolean isValid = limelight.getLatestResult().isValid();

        //AutoAim
        if (gamepad1.x && !xWasPressed) {
            autoAim = !autoAim;
        }
        xWasPressed = gamepad1.x;

        if (autoAim && isValid) {
            double kP = 0.01;
            turn = kP * tx;
            if (Math.abs(tx) < 1.0) {
                turn = 0;
            }
        }

        drive.driveRobotRelative(y, x, turn);

        //Intake Commands
        boolean input3 = gamepad1.left_trigger > 0.3;
        intake.intake(gamepad1.left_bumper, gamepad1.right_bumper, input3);

        //Manual Adjustment
        shooter.setRPM(gamepad1.dpad_up, gamepad1.dpad_down);
        shooter.setHood(gamepad1.dpad_left, gamepad1.dpad_right);
        shooter.RPM(shooter.tyDistance(ty));
        shooter.hood(shooter.tyDistance(ty), autoAim);
        shooter.ShooterGo(gamepad1.y);
        //Lock Turret
        turretServoFront.setPosition(0.5);
        turretServoBack.setPosition(0.5);

//        double distance = Shooter.distance2D(turret.turretpositionX(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()),turret.turretpositionY(follower.getPose().getX(), follower.getPose().getY(),follower.getPose().getHeading()), turret.redGoalX,turret.redGoalY);


        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("RPM", shooter.targetRPM);
        telemetry.addData("Current RPM", shooter.avgRPM);
        telemetry.addData("hood", shooter.targetHood);
        telemetry.addData("Yaw", odom.getYawScalar());
        telemetry.addData("heading from odom",  odom.getHeading(AngleUnit.DEGREES));
        telemetry.addData("tx", limelight.getLatestResult().getTx());
        telemetry.addData("ty", limelight.getLatestResult().getTy());
        telemetry.addData("distance calculated", shooter.tyDistance(ty));
        telemetry.addData("has Target?", limelight.getLatestResult().isValid());
        telemetry.addData("Calculated RPM", shooter.RPM(shooter.tyDistance(ty)));
        telemetry.addData("Calculated Hood", shooter.hood(shooter.tyDistance(ty), limelight.getLatestResult().isValid()));
    }
}
