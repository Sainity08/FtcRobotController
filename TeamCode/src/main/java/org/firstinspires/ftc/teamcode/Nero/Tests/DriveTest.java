package org.firstinspires.ftc.teamcode.Nero.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Nero.Mecanisms.Drivetrain;

@TeleOp
public class DriveTest extends OpMode {

    Drivetrain drive = new Drivetrain();
    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        drive.driveRobotRelative(y, x, turn);
    }
}
