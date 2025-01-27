package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc6205.globals.AutoPresets;
import org.firstinspires.ftc.teamcode.ftc6205.globals.RobotConfiguration;

@Config
public class Claw {
    Servo claw;
    String name;
    HardwareMap hwMap;
    Gamepad gpad1;
    Telemetry telem;
    public Claw (String aname, HardwareMap ahwMap, Gamepad gpad, Telemetry atelem) {
        name = aname;
        hwMap = ahwMap;
        gpad1 = gpad;
        telem = atelem;
        initClaw();
    }

    public void initClaw()  {
        //hwMap = ahwMap;

        claw = hwMap.servo.get(name);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(AutoPresets.claw_pinch); // pinch
    }

    public void grab(Gamepad gpad) {
        gpad1 = gpad;
        if (    (gpad1.right_bumper) ||
                (gpad1.right_bumper  && gpad1.dpad_up) ||
                (gpad1.right_bumper  && gpad1.dpad_down)
            )
        {
            claw.setPosition(AutoPresets.claw_release);
        } else {
            claw.setPosition(AutoPresets.claw_pinch);
        }
    }

    public void setPosition(double pos) {
        claw.setPosition(pos);
    }
}
