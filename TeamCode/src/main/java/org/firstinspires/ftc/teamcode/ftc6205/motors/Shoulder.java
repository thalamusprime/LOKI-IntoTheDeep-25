package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc6205.globals.AutoPresets;
import org.firstinspires.ftc.teamcode.ftc6205.globals.RobotConfiguration;

@Config
public class Shoulder {
    Servo shoulder;
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
        shoulder = hwMap.servo.get(RobotConfiguration.shoulder_name);
        shoulder.setDirection(Servo.Direction.REVERSE);
        shoulder.setPosition(AutoPresets.shoulder_rest); // reset
    }

    public void shrug(Gamepad gpad) {
        gpad1 = gpad;
        if (gpad1.left_bumper && gpad1.ps) {
            initShoulder(hwMap);
            shoulder.setPosition(AutoPresets.shoulder_rest);
        }
        else if (gpad1.left_bumper && gpad1.dpad_up) {
            shoulder.setPosition(AutoPresets.shoulder_wall);
        }
        else if (gpad1.left_bumper && gpad1.dpad_left) {
            shoulder.setPosition(AutoPresets.shoulder_specimen_hook);
        }
        else if (gpad1.left_bumper && gpad1.dpad_right) {
            shoulder.setPosition(AutoPresets.shoulder_specimen_stage);
        }
        else if (gpad1.left_bumper && gpad1.dpad_down) {
            shoulder.setPosition(AutoPresets.shoulder_floor);
        }

        else if (!gpad1.left_bumper && gpad1.dpad_up) {
            shoulder.setPosition(shoulder.getPosition() - AutoPresets.shoulder_inc);
        }
        else if (!gpad1.left_bumper && gpad1.dpad_down) {
            shoulder.setPosition(shoulder.getPosition() + AutoPresets.shoulder_inc);
        }

    }

    public void setPosition(double pos) {
        shoulder.setPosition(pos);
    }
    public double getPosition() {
        return shoulder.getPosition();
    }
}
