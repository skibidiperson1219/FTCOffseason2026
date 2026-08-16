package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Test", group = "Test")
public class FlywheelTest extends LinearOpMode {
    public DcMotorEx flywheelMotorLeft;
    public DcMotorEx flywheelMotorRight;
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");

        flywheelMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        flywheelMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.a) {
                flywheelMotorLeft.setPower(1);
                flywheelMotorRight.setPower(1);
            } else {
                flywheelMotorLeft.setPower(0);
                flywheelMotorRight.setPower(0);
            }

            double leftTPS = flywheelMotorLeft.getVelocity();
            double rightTPS = flywheelMotorRight.getVelocity();

            double leftRPM = (leftTPS / TICKS_PER_REV) * 60.0;
            double rightRPM = (rightTPS / TICKS_PER_REV) * 60.0;

            telemetry.addData("Left TPS", "%.2f", leftTPS);
            telemetry.addData("Right TPS", "%.2f", rightTPS);

            telemetry.addData("Left RPM", "%.2f", leftRPM);
            telemetry.addData("Right RPM", "%.2f", rightRPM);

            telemetry.update();
        }
    }
}
