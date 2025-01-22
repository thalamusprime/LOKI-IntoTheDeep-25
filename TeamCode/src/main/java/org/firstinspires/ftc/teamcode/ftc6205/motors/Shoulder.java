package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shoulder {
    Servo servo;
    HardwareMap hwMap;
    Gamepad gpad1;
    double shoulder_pos;

    public static double shrug_0 = 0.0;
    public static double shrug_specimen_stage = 0.12; //0.1 straight up
    public static double shrug_specimen_hook = 0.16; //0.1 straight up
    public static double shrug_floor = 0.25;
    public static double shoulder_inc = 0.001;

    public Shoulder(HardwareMap ahwMap) {
        initShoulder(ahwMap);
    }

    public void initShoulder(HardwareMap ahwMap)  {
        hwMap = ahwMap;
        servo = hwMap.servo.get("shoulder");
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(shrug_0); // reset
    }

    public void shrug(Gamepad gpad) {
        gpad1 = gpad;
        if (gpad1.left_bumper && gpad1.ps) {
            initShoulder(hwMap);
            //shoulder.setPosition(shrug_0);
        }
        else if (gpad1.left_bumper && gpad1.dpad_left) {
            servo.setPosition(shrug_specimen_stage);
        }
        else if (gpad1.left_bumper && gpad1.dpad_right) {
            servo.setPosition(shrug_specimen_hook);
        }
        else if (gpad1.left_bumper && gpad1.dpad_down) {
            servo.setPosition(shrug_floor);
        }

        else if (!gpad1.left_bumper && gpad1.dpad_up) {
            double new_pos = servo.getPosition() - shoulder_inc;
            //new_pos =+ shoulder_pos;
            servo.setPosition(new_pos);
        }
        else if (!gpad1.left_bumper && gpad1.dpad_down) {
            double new_pos = servo.getPosition() + shoulder_inc;
            //new_pos =- shoulder_pos;
            servo.setPosition(new_pos);
        }

    }

    public void setPosition(double pos) {
        servo.setPosition(pos);
    }
    public double getPosition() {
        return servo.getPosition();
    }
}
