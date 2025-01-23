package org.firstinspires.ftc.teamcode.ftc6205.globals;
import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoPresets {
    // Claw
    public static double claw_pinch = 0.0; // pinch
    public static double claw_release = 0.1; // drop
    public static int touch_duration = 20;

    public static double wrist_inc = 0.001;
    public static double wrist_floor = 0.0;
    public static double wrist_rest = 0.15;  //0.85;
    public static double wrist_wall = 0.265;
    public static double wrist_hook = 0.200;

    public static double shoulder_inc = 0.001;
    public static double shoulder_rest = 0.0;
    public static double shoulder_wall = 0.220;
    public static double shoulder_specimen_stage = 0.12; //0.1 straight up
    public static double shoulder_specimen_hook = 0.17; //0.1 straight up
    public static double shoulder_floor = 0.245;

}

