package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Wrist {
    Servo wrist;
    HardwareMap hwMap;
    public Wrist(HardwareMap ahwMap){
        initServos(ahwMap);
    }
    private void initServos(HardwareMap ahwMap){
        wrist = hwMap.servo.get("wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.setPosition(0.5);
    }

}
