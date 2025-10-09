package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelServo;
@TeleOp
public class MecanumFlywheelServo extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    FlywheelServo wheel = new FlywheelServo();

    @Override
    public void init() {
        drive.init(hardwareMap);
        FlywheelServo.init(hardwareMap);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean input1 = gamepad1.a;
        boolean input2 = gamepad1.b;
//        boolean input3 = gamepad1.x;

        drive.driveFieldRelative(y, x, turn);
        wheel.doubleA(input1, input2);

    }
}