package org.firstinspires.ftc.teamcode._test.odometry;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.ftc6205.sensors.MicroNavX;

    @Autonomous(name = "TEST - Odometry", group = "TEST")
    //@Disabled
    public class Test_Odometry extends LinearOpMode {

        // The lateral distance between the left and right odometers
        // is called the trackwidth. This is very important for
        // determining angle for turning approximations
        public static final double TRACKWIDTH = 16.615;

        // Center wheel offset is the distance between the
        // center of rotation of the robot and the center odometer.
        // This is to correct for the error that might occur when turning.
        // A negative offset means the odometer is closer to the back,
        // while a positive offset means it is closer to the front.
        public static final double CENTER_WHEEL_OFFSET = -7.25;

        public static final double WHEEL_DIAMETER = 0.846457 * 2.0;
        // if needed, one can add a gearing term here
        public static final double TICKS_PER_REV = 2000;
        public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

        private MotorEx frontLeft, frontRight, backLeft, backRight;
        private MecanumDrive driveTrain;
        private Motor intakeLeft, intakeRight, liftLeft, liftRight;
        private Encoder leftOdometer, rightOdometer, centerOdometer;
        private HolonomicOdometry odometry;
        private IMU imu;
        MicroNavX navx;
        @Override
        public void runOpMode() throws InterruptedException {
            navx = new MicroNavX(hardwareMap);
            //navx.initIMU(hardwareMap);
            // Drive motors
            frontLeft = new MotorEx(hardwareMap, "frontleft");
            frontRight = new MotorEx(hardwareMap, "frontright");
            backLeft = new MotorEx(hardwareMap, "backleft");
            backRight = new MotorEx(hardwareMap, "backright");
            driveTrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

            // Here we set the distance per pulse of the odometers.
            // This is to keep the units consistent for the odometry.
            leftOdometer = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
            rightOdometer = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
            centerOdometer = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
            rightOdometer.setDirection(Motor.Direction.REVERSE);

            odometry = new HolonomicOdometry(
                    leftOdometer::getDistance,
                    rightOdometer::getDistance,
                    centerOdometer::getDistance,
                    TRACKWIDTH, CENTER_WHEEL_OFFSET
            );

            // read the current position from the position tracker
            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();

            telemetry.addData("Robot Position at Init: ", PositionTracker.robotPose);
            telemetry.update();
            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                if (gamepad1.options) {
                    navx.resetYaw();
                    leftOdometer.reset();
                    centerOdometer.reset();
                    rightOdometer.reset();
                }

                float triggerSpeed = gamepad1.right_trigger;
                driveTrain.driveFieldCentric(
                        -gamepad1.left_stick_x * (0.3 + triggerSpeed * 0.7),
                        gamepad1.left_stick_y * (0.3 + triggerSpeed * 0.7),
                        -gamepad1.right_stick_x * (0.3 + triggerSpeed * 0.7),
                        navx.getYawInDegrees(),
                        false
                );

                odometry.updatePose(); // update the position
                PositionTracker.robotPose = odometry.getPose();

                telemetry.addLine(String.format(
                        "DISTANCE  %5.2f %5.2f %5.2f",
                        leftOdometer.getDistance(),// * 0.003, // 0.0075
                        centerOdometer.getDistance(),// * 0.003,
                        rightOdometer.getDistance() //* 0.003
                ));

                telemetry.addLine(String.format(
                        "POSITION  %3d",
                        navx.getYawInDegrees()
                ));
                telemetry.update();
            }
        }

    }

