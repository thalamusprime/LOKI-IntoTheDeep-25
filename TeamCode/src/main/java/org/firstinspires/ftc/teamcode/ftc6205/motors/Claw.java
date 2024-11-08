package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo claw;
    HardwareMap hwMap;
    public void init(HardwareMap ahwMap)  {
        hwMap = ahwMap;

        claw = hwMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(0.0); // pinch
    }

    public void setPosition(double pos) {
        claw.setPosition(pos);
    }
}
