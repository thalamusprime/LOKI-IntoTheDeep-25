package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ftc6205.globals.AUTOConstants;
@Config
public class Claw {
    Servo claw;
    HardwareMap hwMap;
    Gamepad gpad1;

    public Claw (HardwareMap ahwMap) {
        initClaw(ahwMap);
    }

    public void initClaw(HardwareMap ahwMap)  {
        hwMap = ahwMap;

        claw = hwMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(0.0); // pinch
    }

    public void grab(Gamepad gpad) {
        gpad1 = gpad;
        if (    (gpad1.right_bumper) ||
                (gpad1.right_bumper  && gpad1.dpad_up) ||
                (gpad1.right_bumper  && gpad1.dpad_down)
            )
        {
            claw.setPosition(AUTOConstants.claw_release);
        } else {
            claw.setPosition(AUTOConstants.claw_pinch);
        }
    }

    public void setPosition(double pos) {
        claw.setPosition(pos);
    }
}
