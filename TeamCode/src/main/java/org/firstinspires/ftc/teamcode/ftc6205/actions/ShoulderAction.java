package org.firstinspires.ftc.teamcode.ftc6205.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc6205.globals.AutoPresets;

public class ShoulderAction {
    private Servo shoulder;

    public ShoulderAction(HardwareMap hardwareMap) {
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        shoulder.setDirection(Servo.Direction.REVERSE);
    }

    //todo RestShoulder ---------------------------------------------------
    public class RestShoulder implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //packet.clearLines();
            packet.put("Shoulder: ", "Init");
            shoulder.setPosition(AutoPresets.shoulder_rest);
            return false;
        }
    }
    public Action restShoulder() {
        return new RestShoulder();
    }

    //todo WallShoulder ---------------------------------------------------
    public class WallShoulder implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            packet.put("Shoulder: ", "Stage");
            shoulder.setPosition(AutoPresets.shoulder_wall);
            return false;
        }
    }
    public Action wallShoulder() {
        return new WallShoulder();
    }

    //todo StageShoulder ---------------------------------------------------
    public class StageShoulder implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            packet.put("Shoulder: ", "Stage");
            shoulder.setPosition(AutoPresets.shoulder_specimen_stage);
            return false;
        }
    }
    public Action stageShoulder() {
        return new StageShoulder();
    }

    //todo FloorShoulder ---------------------------------------------------
    public class FloorShoulder implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //packet.clearLines();
            packet.put("Shoulder: ", "Floor");
            shoulder.setPosition(AutoPresets.shoulder_floor);
            return false;
        }
    }
    public Action floorShoulder() {
        return new FloorShoulder();
    }

    //todo HookShoulder ---------------------------------------------------
    public class HookShoulder implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //packet.clearLines();
            packet.put("Shoulder: ", "Hook");
            shoulder.setPosition(AutoPresets.shoulder_specimen_hook);
            return false;
        }
    }
    public Action hookShoulder() {
        return new HookShoulder();
    }

}
