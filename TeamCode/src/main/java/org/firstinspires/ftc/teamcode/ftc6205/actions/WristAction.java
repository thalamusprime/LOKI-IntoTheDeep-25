package org.firstinspires.ftc.teamcode.ftc6205.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc6205.globals.AutoPresets;

public class WristAction {
    private Servo wrist;

    public WristAction(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(AutoPresets.wrist_start);
    }

    //todo RestWrist ---------------------------------------------------
    public class RestWrist implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //packet.clearLines();
            packet.put("Wrist: ", "Rest");
            wrist.setPosition(AutoPresets.wrist_start);
            return false;
        }
    }
    public Action restWrist() {
        return new RestWrist();
    }

    //todo FloorWrist ---------------------------------------------------
    public class FloorWrist implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            packet.put("Wrist: ", "Floor");
            wrist.setPosition(AutoPresets.wrist_floor);
            return false;
        }
    }
    public Action floorWrist() {
        return new FloorWrist();
    }

    //todo WallWrist ---------------------------------------------------
    public class WallWrist implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            packet.put("Wrist: ", "Wall");
            wrist.setPosition(AutoPresets.wrist_wall);
            return false;
        }
    }
    public Action wallWrist() {
        return new WallWrist();
    }

    //todo HookWrist ---------------------------------------------------
    public class HookWrist implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            packet.put("Wrist: ", "Hook");
            wrist.setPosition(AutoPresets.wrist_hook);
            return false;
        }
    }
    public Action hookWrist() {
        return new HookWrist();
    }

    //todo WallWrist ---------------------------------------------------
    public class PinchWrist implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            packet.put("Wrist: ", "Pinch");
            wrist.setPosition(AutoPresets.wrist_pinch);
            return false;
        }
    }
    public Action pinchWrist() {
        return new PinchWrist();
    }

}

