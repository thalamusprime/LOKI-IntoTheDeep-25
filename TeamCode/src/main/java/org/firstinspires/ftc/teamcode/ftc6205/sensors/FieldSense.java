package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import static org.firstinspires.ftc.teamcode.ftc6205.globals.AutoPresets.touch_duration;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class FieldSense {
    public TouchSensor touchARM;
    public boolean touchValue;
    //public double touchState;

    HardwareMap hwMap;
    Gamepad gpad1;

    public FieldSense(HardwareMap ahwMap){
        initFieldSense(ahwMap);
    }

    public void initFieldSense(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // Encoders
        touchARM = hwMap.touchSensor.get("touchArm");
    }

    public boolean check(Gamepad gpad) throws InterruptedException {
        gpad1 = gpad;
        touchValue = touchARM.isPressed();
        if (touchValue) {
            gpad1.rumble(touch_duration);
        }
        return touchValue;
    }


}
