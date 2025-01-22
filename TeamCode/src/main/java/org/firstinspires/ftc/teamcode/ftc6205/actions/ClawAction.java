package org.firstinspires.ftc.teamcode.ftc6205.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ftc6205.globals.AUTOConstants;

public class ClawAction {
    private Servo claw;
    public ClawAction(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            packet.clearLines();
            packet.put("Claw: ", "CLOSE");
            claw.setPosition(AUTOConstants.claw_pinch);
            return false;
        }
    }
    public Action closeClaw() {
        return new CloseClaw();
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            packet.clearLines();
            packet.put("Claw: ", "OPEN");
            claw.setPosition(AUTOConstants.claw_release);
            return false;
        }
    }
    public Action openClaw() {
        return new OpenClaw();
    }
}
