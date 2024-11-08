package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Touch {
    public TouchSensor touchARM;
    public boolean touchValue;
    //public double touchState;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // Encoders
        touchARM = hwMap.touchSensor.get("touchArm");
    }

    public boolean isPressed() throws InterruptedException {
        // Get current encoder position
        touchValue = touchARM.isPressed();
        return touchValue;
    }


}
