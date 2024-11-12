package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ftc6205.globals.VisionConstants;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TEST - Cube Vision", group = "TEST")
@Disabled
public class SampleVision extends LinearOpMode {
    HardwareMap hMap;

    double cX = 0;
    double cY = 0;
    double width = 0;
    static String cube_spotted = "";

    public OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    // Camera: C270
    //private static final int CAMERA_WIDTH = 1280;//640; // width  of wanted camera resolution
    //private static final int CAMERA_HEIGHT = 720;//480; // height of wanted camera resolution
    // Camera: C920
    public static final int CAMERA_WIDTH = 1920; //640; // width  of wanted camera resolution
    public static final int CAMERA_HEIGHT = 1080; //480; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    String token_color = "";

    @Override
    public void runOpMode() {
        // initOpenCV
        try {
            initOpenCV(hMap, token_color);
        } catch (Exception e) {
            telemetry.addLine("Exception thrown in initOpenCV: %s" + e);
        }

        // Webcam 1
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance (inches)", (getDistance(width)));
            telemetry.addData("Location", cube_spotted);
            telemetry.addData("Color", token_color);
            telemetry.update();
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    public void initOpenCV(HardwareMap hMap, String cube_color) throws InterruptedException {
        // Assign token_color
        token_color = cube_color;

        // Create an instance of the camera
        int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                //hardwareMap.get(WebcamName.class, "Webcam 1"),
                hMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());
        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        Mat yellowMask;
        Mat hierarchy;
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                if (cX < 500) {cube_spotted="LEFT";}
                if (cX > 500 && cX < 1500) {cube_spotted="MIDDLE";}
                if (cX > 1500) {cube_spotted="RIGHT";}

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
            }

            return input;
        }

        //@NonNull
        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            // TODO: mask top half of frame to reduce noise
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            /**
             * blue: 0 - 50
             * green: 50 - 75
             * yellow: 75 - 100
             * red: 100 - 150
             * white: ?
             * purple: ?
             */
            Scalar lowerYellow;
            Scalar upperYellow;
            if (token_color == "BLUE") {
                lowerYellow = new Scalar(VisionConstants.Blue_Hue_low, 50, 100); //Sat: 100
                upperYellow = new Scalar(VisionConstants.Blue_Hue_high, 255, 255); //Sat: 255
            } else {
                lowerYellow = new Scalar(VisionConstants.Red_Hue_low, 50, 100); //Sat: 100
                upperYellow = new Scalar(VisionConstants.Red_Hue_high, 255, 255); //Sat: 255
            }

            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        //@NonNull
        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

    public String getCube_spotted(){
        return cube_spotted;
    }
}
