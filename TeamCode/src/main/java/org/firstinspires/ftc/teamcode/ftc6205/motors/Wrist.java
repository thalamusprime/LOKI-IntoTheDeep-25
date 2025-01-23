package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc6205.globals.AutoPresets;
import org.firstinspires.ftc.teamcode.ftc6205.globals.RobotConfiguration;

@Config
public class Wrist {
    Servo servo;
    final static String name = RobotConfiguration.wrist_name;
    final HardwareMap hwMap;
    Gamepad gpad1;
    final Telemetry telemetry;

    public static double wrist_floor = 0.0;
    public static double wrist_start = 0.15;  //0.85;
    public static double wrist_down_arm_in = 0.4;
    public static double wrist_down_arm_out = 0.45;
    double position;
    public static double wrist_inc = 0.001;

    public Wrist(HardwareMap ahwMap, Gamepad gpad, Telemetry telem){
        hwMap = ahwMap;
        gpad1 = gpad;
        telemetry = telem;
        this.initWrist();
    }

    public void initWrist(){
        servo = hwMap.servo.get(name);
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(AutoPresets.wrist_rest);
    }

    public void revolve(Gamepad gpad) {
        gpad1 = gpad;
        //PS: TRIANGLE
        if ( gpad1.left_bumper && gpad1.y ) {
            position = AutoPresets.wrist_wall;
            servo.setPosition(position);
        } //PS: CIRCLE
        if ( gpad1.left_bumper && gpad1.b ) {
            position = AutoPresets.wrist_rest;
            servo.setPosition(position);
        } //PS: SQUARE
        if ( gpad1.left_bumper && gpad1.x ) {
            position = AutoPresets.wrist_hook;
            servo.setPosition(position);
        } //PS: CROSS
        if ( gpad1.left_bumper && gpad1.a ) {
            position = AutoPresets.wrist_floor;
            servo.setPosition(position);
        }

        else if (!gpad1.left_bumper && gpad1.dpad_left) {
            double new_pos = servo.getPosition() + wrist_inc;
            //new_pos =+ shoulder_pos;
            servo.setPosition(new_pos);
        }
        else if (!gpad1.left_bumper && gpad1.dpad_right) {
            double new_pos = servo.getPosition() - wrist_inc;
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
