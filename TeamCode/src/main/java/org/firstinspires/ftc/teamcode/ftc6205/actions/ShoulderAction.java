package org.firstinspires.ftc.teamcode.ftc6205.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc6205.globals.AUTOConstants;

public class ShoulderAction {
    private Servo shoulder;

    public ShoulderAction(HardwareMap hardwareMap) {
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        shoulder.setDirection(Servo.Direction.REVERSE);

    }

    //todo InitShoulder ---------------------------------------------------
    public class InitShoulder implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //packet.clearLines();
            packet.put("Shoulder: ", "Init");
            shoulder.setPosition(AUTOConstants.shrug_0);
            return false;
        }
    }
    public Action initShoulder() {
        return new InitShoulder();
    }

    //todo FloorShoulder ---------------------------------------------------
    public class StageShoulder implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            packet.put("Shoulder: ", "Stage");
            shoulder.setPosition(AUTOConstants.shrug_specimen_stage);
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
            shoulder.setPosition(AUTOConstants.shrug_floor);
            return false;
        }
    }
    public Action floorShoulder() {
        return new FloorShoulder();
    }

    //todo FloorShoulder ---------------------------------------------------
    public class HookShoulder implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //packet.clearLines();
            packet.put("Shoulder: ", "Hook");
            shoulder.setPosition(AUTOConstants.shrug_specimen_hook);
            return false;
        }
    }
    public Action hookShoulder() {
        return new HookShoulder();
    }


}
