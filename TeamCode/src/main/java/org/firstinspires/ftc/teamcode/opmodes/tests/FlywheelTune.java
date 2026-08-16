package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Flywheel Tuner", group = "Test")
public class FlywheelTune extends OpMode{
    public DcMotorEx flywheelMotorLeft;
    public DcMotorEx flywheelMotorRight;

    public double highVelocity = 2000;
    public double lowVelocity = 1000;

    double curTargetVelocity = highVelocity;

    double F=0;
    double P=0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};

    int stepIndex = 1; // change at by 1

    @Override
    public void init() {
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");

        flywheelMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        // get all our gamepad commands
        // set target velocity
        // update telemetry

        // Press Y to toggle between high/low speed
        if (gamepad2.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        // Press B to change step sizes
        if (gamepad2.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Adjust F
        if (gamepad2.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];

        }
        if (gamepad2.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        // Adjust P
        if (gamepad2.dpadDownWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad2.dpadUpWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // set velocity
        flywheelMotorLeft.setVelocity(curTargetVelocity);
        flywheelMotorRight.setVelocity(curTargetVelocity);

        double leftVelocity = flywheelMotorLeft.getVelocity();
        double rightVelocity = flywheelMotorRight.getVelocity();
        double avgVelocity = (leftVelocity + rightVelocity) / 2.0;

        // error
        double error = curTargetVelocity - avgVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity (AV)", "%.2f", avgVelocity);
        telemetry.addData("Error", "%.2f",error);
        telemetry.addData("raw power left", flywheelMotorLeft.getPower());
        telemetry.addData("raw power right", flywheelMotorRight.getPower());
        telemetry.addLine("-----------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }
}