package org.firstinspires.ftc.teamcode.ftc6205.logging;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Shoulder;
import org.firstinspires.ftc.teamcode.ftc6205.motors.Wrist;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.DeadWheels;
import org.firstinspires.ftc.teamcode.ftc6205.sensors.FieldSense;

//@TeleOp
@Disabled
public class DSTelemetry {
    HardwareMap ahwMap;
    Telemetry telemetry;

    //DriveEncoders driveEncoders = new DriveEncoders(ahwMap);

    String tag4_x = "";
    String tag4_y = "";
    String tag4_z = "";
    String tag5_x = "";
    String tag5_y = "";
    String tag5_z = "";
    String tag6_x = "";
    String tag6_y = "";
    String tag6_z = "";
    String tag7_x = "";
    String tag7_y = "";
    String tag7_z = "";
    String tag8_x = "";
    String tag8_y = "";
    String tag8_z = "";

    // Send telemetry data
    public void sendTelemetry(
            Telemetry telemetry,
            DeadWheels driveEncoders,
            Wrist wrist,
            Shoulder shoulder
            //FieldSense fieldSenseArm
    ) throws InterruptedException {

        telemetry.addLine(String.format(
                "ENCODER L|B|R %5.2f %5.2f %5.2f",
                driveEncoders.encLeftValue, // 0.0075
                driveEncoders.encBackValue,
                driveEncoders.encRightValue
        ));
        telemetry.addLine(String.format(
                "WRIST: %5.3f",
                wrist.getPosition()
        ));
        telemetry.addLine(String.format(
                "SHOULDER: %5.3f",
                shoulder.getPosition()
        ));
        //telemetry.addLine(String.valueOf(fieldSenseArm.touchValue));

//    public void sendTelemetry(
//                Telemetry telemetry,
//        double encLeftValue,
//        double encBackValue,
//        double encRightValue
//        ) throws InterruptedException {
//
//            telemetry.addLine(String.format(
//                    "ENCODER L|B|R %5.2f %5.2f %5.2f",
//                    encLeftValue * 0.003, // 0.0075
//                    encBackValue * 0.003,
//                    encRightValue * 0.003
//            ));

        //Vision Processing
//        int tags = tagProcessor.getDetections().size();
//        if (tagProcessor.getDetections().size() > 0) {
//            ArrayList tagList = tagProcessor.getDetections();
//            if (tagList != null) {
//                for (Object tagItem: tagList) {
//                    AprilTagDetection tag = (AprilTagDetection) tagItem;
//                    if (tag.id == 4) {
//                        tag4_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag4_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag4_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                    if (tag.id == 5) {
//                        tag5_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag5_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag5_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                    if (tag.id == 6) {
//                        tag6_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag6_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag6_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                    if (tag.id == 7) {
//                        tag7_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag7_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag7_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                    if (tag.id == 8) {
//                        tag8_x = String.format("%7.2f", tag.ftcPose.x);
//                        tag8_y = String.format("%7.2f", tag.ftcPose.y);
//                        tag8_z = String.format("%7.2f", tag.ftcPose.z);
//                    }
//                }
//            }
//            telemetry.addLine("---");
//            telemetry.addLine("=== BLUE");
//            telemetry.addLine("ID (1)" + tag1_x +  tag1_y + tag1_z);
//            telemetry.addLine("ID (2)" + tag2_x +  tag2_y + tag2_y);
//            telemetry.addLine("ID (3)" + tag3_x +  tag3_y + tag3_z);
//            telemetry.addLine("---");
//            telemetry.addLine("ID (9)" + tag9_x +  tag9_y + tag9_z);
//            telemetry.addLine("ID (10)" + tag10_x +  tag10_y + tag10_z);
//            telemetry.addLine("---");
//
//            telemetry.addLine("---");
//            telemetry.addLine("=== RED");
//            telemetry.addLine("ID (4)" + tag4_x +  tag4_y + tag4_z);
//            telemetry.addLine("ID (5)" + tag5_x +  tag5_y + tag5_z);
//            telemetry.addLine("ID (6)" + tag6_x +  tag6_y + tag6_z);
//            telemetry.addLine("---");
//            telemetry.addLine("ID (7)" + tag7_x +  tag7_y + tag7_z);
//            telemetry.addLine("ID (8)" + tag8_x +  tag8_y + tag8_z);
//        }
        telemetry.update();
    }}
