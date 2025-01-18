package org.firstinspires.ftc.teamcode.ftc6205.globals;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FieldCoordinates {
    public static final double NORTH = 0;
    public static final double WEST = Math.toRadians(90.0);//90;
    public static final double EAST = Math.toRadians(-90.0);
    public static final double SOUTH = Math.toRadians(180.0);

    public static final double PanelWidth = 23.5;

    public static final double A = (PanelWidth / 2) * 5; //60
    public static final double B = (PanelWidth / 2) * 3; //36;
    public static final double C = (PanelWidth / 2) * 1; //12;
    public static final double D = (PanelWidth / 2) * -1; //-12;
    public static final double E = (PanelWidth / 2) * -3; //-36;
    public static final double F = (PanelWidth / 2) * -5; //-60;
    public static final double VI = (PanelWidth / 2) * 5; //60;
    public static final double V = (PanelWidth / 2) * 3; //36;
    public static final double IV = (PanelWidth / 2) * 1; //12;
    public static final double III = (PanelWidth / 2) * -1; //-12;
    public static final double II = (PanelWidth / 2) * -3; //-36;
    public static final double I = (PanelWidth / 2) * -5; //-60;
}
