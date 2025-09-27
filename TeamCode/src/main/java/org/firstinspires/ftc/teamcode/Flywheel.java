package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Flywheel {

    private DcMotor Flywheel;

    public void init(HardwareMap hwMap) {
        Flywheel = hwMap.get(DcMotor.class, "flywheel");
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}
