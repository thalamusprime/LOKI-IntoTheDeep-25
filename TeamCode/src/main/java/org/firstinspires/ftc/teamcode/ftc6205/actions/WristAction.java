package org.firstinspires.ftc.teamcode.ftc6205.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc6205.globals.AUTOConstants;

public class WristAction {
    private Servo wrist;

    public WristAction(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    //todo InitWrist ---------------------------------------------------
    public class InitWrist implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //packet.clearLines();
            packet.put("Wrist: ", "Init");
            wrist.setPosition(AUTOConstants.wrist_0);
            return false;
        }
    }
    public Action initWrist() {
        return new InitWrist();
    }

    //todo FloorWrist ---------------------------------------------------
    public class FloorWrist implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //packet.clearLines();
            packet.put("Wrist: ", "Floor");
            wrist.setPosition(AUTOConstants.wrist_floor);
            return false;
        }
    }
    public Action floorWrist() {
        return new FloorWrist();
    }
}
