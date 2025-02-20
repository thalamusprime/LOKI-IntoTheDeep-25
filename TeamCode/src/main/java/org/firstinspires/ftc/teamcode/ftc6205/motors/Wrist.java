package org.firstinspires.ftc.teamcode.ftc6205.motors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc6205.globals.AutoPresets;

@Config
public class Wrist {
    Servo servo;
    String name;
    final HardwareMap hwMap;
    Gamepad gpad1;
    final Telemetry telemetry;

    double position;
    //public static double wrist_inc = 0.001;

    public Wrist(String aname, HardwareMap ahwMap, Gamepad gpad, Telemetry telem){
        name = aname;
        hwMap = ahwMap;
        gpad1 = gpad;
        telemetry = telem;
        this.initWrist();
    }
    public void initWrist(){
        servo = hwMap.servo.get(name);
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(AutoPresets.wrist_start);
    }
    public void revolve(Gamepad gpad) {
        gpad1 = gpad;
        //PS: TRIANGLE
        if ( gpad1.left_bumper && gpad1.y ) {
            position = AutoPresets.wrist_wall;
            servo.setPosition(position);
        } //PS: CIRCLE
        if ( gpad1.left_bumper && gpad1.b ) {
            position = AutoPresets.wrist_start;
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

        if (!gpad1.left_bumper && gpad1.dpad_left) {
            double new_pos = servo.getPosition() + AutoPresets.wrist_inc;
            servo.setPosition(new_pos);
        }
        else if (!gpad1.left_bumper && gpad1.dpad_right) {
            double new_pos = servo.getPosition() - AutoPresets.wrist_inc;
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
