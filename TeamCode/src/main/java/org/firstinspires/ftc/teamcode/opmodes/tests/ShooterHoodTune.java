package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.opmodes.teleop.MainTele.START_X;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.MainTele.START_Y;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.MainTele.START_HEADING;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Shooter Hood Tune", group = "Test")
public class ShooterHoodTune extends LinearOpMode {

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;
    private Servo hood;
    private Intake intake;
    private GoBildaPinpointDriver pinpoint;

    private static final double GOAL_X = -72;
    private static final double GOAL_Y = -72;

    private double targetVelocity = 1000; // ticks per second
    private double hoodPosition = 0.33;

    private static final double P = 40;
    private static final double F = 12.64;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(127.52, -68.1, DistanceUnit.MM);
        pinpoint.setEncoderResolution(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD,
                com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        flywheelLeft.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );

        flywheelRight.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );

        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);
            double distance = Math.hypot(GOAL_X - robotX, GOAL_Y - robotY);

            // Flywheel speed tuning
            if (gamepad2.dpad_up) {
                targetVelocity += 10;
                sleep(100);
            }

            if (gamepad2.dpad_down) {
                targetVelocity -= 10;
                sleep(100);
            }

            // Hood tuning
            if (gamepad2.dpadRightWasPressed() ) {
                hoodPosition += 0.01;
            }

            if (gamepad2.dpadLeftWasPressed()) {
                hoodPosition -= 0.01;
            }

            hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition)); // clamp 0-1

            if (gamepad2.a) {
                flywheelLeft.setVelocity(targetVelocity);
                flywheelRight.setVelocity(targetVelocity);
            } else if (gamepad2.b) {
                flywheelLeft.setVelocity(0);
                flywheelRight.setVelocity(0);
            }

            hood.setPosition(hoodPosition);

            if (gamepad2.left_trigger > 0.2) {
                intake.intake();
            } else if (gamepad2.left_bumper) {
                intake.reverse();
            } else {
                intake.stop();
            }
            intake.update();

            double leftVelocity = flywheelLeft.getVelocity();
            double rightVelocity = flywheelRight.getVelocity();
            double averageVelocity = (leftVelocity + rightVelocity) / 2.0;

            telemetry.addData("Distance to Goal", "%.2f in", distance);
            telemetry.addLine();
            telemetry.addData("Target Velocity", "%.2f ticks/sec", targetVelocity);
            // telemetry.addData("Left Velocity", "%.2f", leftVelocity);
            // telemetry.addData("Right Velocity", "%.2f", rightVelocity);
            telemetry.addData("Average Velocity", "%.2f", averageVelocity);
            telemetry.addData("Velocity Error", "%.2f", targetVelocity - averageVelocity);
            telemetry.addData("Hood Position", "%.3f", hoodPosition);
            telemetry.addLine();
            telemetry.addLine("A = spin flywheel");
            telemetry.addLine("B = stop flywheel");
            telemetry.addLine("Dpad Up/Down = change velocity");
            telemetry.addLine("Dpad Left/Right = change hood");
            telemetry.update();
        }
    }
}
