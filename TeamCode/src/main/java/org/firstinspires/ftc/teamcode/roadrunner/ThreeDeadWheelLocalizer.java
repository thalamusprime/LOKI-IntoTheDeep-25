package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.messages.ThreeDeadWheelInputsMessage;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double leftYTicks = -2673.2800770924023;//.00566;//285; // 0.0 y position of the first parallel encoder (in tick units)
        public double rightYTicks =  2663.77639896873; // 1.0 y position of the second parallel encoder (in tick units)
        public double backXTicks = -2096.269842665058; // 0.0 x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder leftEncoder, rightEncoder, backEncoder;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontleft")));
        rightEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontright")));
        backEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backleft")));

        // TODO: reverse encoder directions if needed
        leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = leftEncoder.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = rightEncoder.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = backEncoder.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.leftYTicks * par1PosDelta - PARAMS.rightYTicks * par0PosDelta) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                                (PARAMS.leftYTicks * par1PosVel.velocity - PARAMS.rightYTicks * par0PosVel.velocity) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.backXTicks / (PARAMS.leftYTicks - PARAMS.rightYTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.backXTicks / (PARAMS.leftYTicks - PARAMS.rightYTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.leftYTicks - PARAMS.rightYTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }
}
