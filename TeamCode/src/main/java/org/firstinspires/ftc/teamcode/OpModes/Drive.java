package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;

@TeleOp
public class Drive extends OpMode {
    Drivetrain drive = new Drivetrain();
    Intake intake = new Intake();

    private DcMotor NSIntake, SIntake;


    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        NSIntake = hardwareMap.get(DcMotor.class, "intake1");
        SIntake = hardwareMap.get(DcMotor.class, "intake2");
        NSIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean input1 = gamepad1.a;
        boolean input3 = gamepad1.b;

        drive.driveRobotRelative(y,x,turn);
        intake.NonStationary(input1);
        intake.stationary(input3);

    }
}