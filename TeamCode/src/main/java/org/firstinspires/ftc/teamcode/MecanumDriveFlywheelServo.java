package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MecanumDriveFlywheelServo extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Flywheel wheel = new Flywheel();
    private static CRServo leftServo;
    private static CRServo rightServo;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean buttonPressed = false;
    private boolean movementStarted = false;
    private boolean movementCompleted = false;



    @Override
    public void init() {
        drive.init(hardwareMap);
        Flywheel.init(hardwareMap);
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        rightServo.setDirection(CRServo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean input1 = gamepad1.a;
        boolean input2 = gamepad1.x;

        if (gamepad1.b && !buttonPressed) {
            buttonPressed = true;
            timer.reset();
            movementStarted = true;
            movementCompleted = false;
        }

        if (movementStarted && !movementCompleted) {
            if (timer.seconds() < 1.0) { //change value its in (sec)
                leftServo.setPower(1.0);
                rightServo.setPower(1.0);
            } else {
                leftServo.setPower(0.0);
                rightServo.setPower(0.0);
                movementCompleted = true;
                movementStarted = false;
            }
        }

        drive.driveFieldRelative(y, x, turn);
        wheel.spin(input1,input2);

    }
}
