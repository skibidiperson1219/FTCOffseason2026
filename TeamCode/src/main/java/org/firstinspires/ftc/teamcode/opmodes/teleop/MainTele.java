package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "MainTeleOp", group = "TeleOp")
public class MainTele extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private GoBildaPinpointDriver pinpoint;

    public static final double START_X = 0.0;
    public static final double START_Y = 0.0;
    public static final double START_HEADING = 0;
    private void resetPinpointPose() {
        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING
                )
        );
    }

    private boolean autoTrack = false;
    private boolean lastLeftBumper = false;
    private boolean slowMode = false;
    private boolean xWasPressed = false;
    private boolean yWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(127.52, -68.1, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        sleep(400);
        waitForStart();
        // put after start because robot may still be moving during setup
        // setting start position is one-time so not in loop
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));

        while (opModeIsActive()) {
            // -----Drivetrain-----
            // Variables that continuously change
            double forward, right, rotate;
            forward = gamepad1.left_stick_y;
            right = -gamepad1.left_stick_x;
            rotate  =  -gamepad1.right_stick_x;

            drivetrain.drive(forward, right, rotate);
            if (gamepad1.x && !xWasPressed) {
                slowMode = !slowMode;
                xWasPressed = true;
            } else if (!gamepad1.x) {
                xWasPressed = false;
            }

            if (slowMode) {
                drivetrain.setDriveSpeedMultiplier(0.5);
            } else {
                drivetrain.setDriveSpeedMultiplier(0.8);
            }

            // -----Turret-----
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);
            double robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

            boolean currentLeftBumper = gamepad2.left_bumper;
            if (currentLeftBumper && !lastLeftBumper) {
                autoTrack = !autoTrack;

                if (!autoTrack) {
                    turret.setTargetAngle(0); // go back to home
                }
            }
            lastLeftBumper = currentLeftBumper;
            if (autoTrack) {
                double aimAngle = turret.autoAim(robotX, robotY, robotHeadingDeg);
                turret.setTargetAngle(aimAngle);
            }
            turret.update();

            // Relocalization
            if (gamepad1.y && !yWasPressed) {
                resetPinpointPose();
                yWasPressed = true;
            } else if (!gamepad1.y) {
                yWasPressed = false;
            }

            // -----Intake and Shooter-----
            if (gamepad2.left_trigger > 0.2) {
                intake.intake();
                shooter.gate.setPosition(shooter.gateClosed);
            } else if (gamepad2.right_trigger > 0.2) {
                intake.reverse();
            } else {
                intake.stop();
            }

            // Math.hypot() is distance formula
            double distance = Math.hypot(Turret.GOAL_X - robotX, Turret.GOAL_Y - robotY);
            shooter.aimForDistance(distance);
            if (gamepad2.a) {
                shooter.requestSpinUp(shooter.getTargetVelocity());
            }
            if (gamepad2.right_bumper) {
                shooter.requestFeed();
            }
            if (gamepad2.b) {
                shooter.requestStop();
            }
            shooter.update();

            if (shooter.shouldRunTransfer()) {
                intake.intake();
            }
            intake.update();
            intake.updateLight();


            telemetry.addLine("---------- INTAKE ----------");
            telemetry.addData("State", intake.getState());
            telemetry.addData("Current Amps", "%.2f", intake.getCurrent());
            telemetry.addData("Estimated Balls", intake.getBallCount());

            telemetry.addLine("---------- SHOOTER ----------");
            telemetry.addData("State", shooter.getState());
            // telemetry.addData("Left Velocity TPS", shooter.getLeftVelocity());
            // telemetry.addData("Right Velocity TPS", shooter.getRightVelocity());
            telemetry.addData("Target Velocity TPS", shooter.getTargetVelocity());
            telemetry.addData("Average Velocity TPS", "%.2f", shooter.getAverageVelocity());
            telemetry.addData("Velocity Error", "%.2f",
                    shooter.getTargetVelocity() - shooter.getAverageVelocity());
            telemetry.addData("Distance", "%.2f", distance);
            telemetry.addData("Calculated Velocity",
                    shooter.getFlywheelSpeed(distance));
            telemetry.addData("Calculated Hood",
                    shooter.getHoodAngle(distance));

            telemetry.addLine("---------- TURRET ----------");
            telemetry.addData("Robot X", "%.2f", robotX);
            telemetry.addData("Robot Y", "%.2f", robotY);
            telemetry.addData("Robot Heading", "%.2f", robotHeadingDeg);
            telemetry.addData("Current Angle", "%.2f", turret.getCurrentAngle());
            telemetry.addData("Target Angle", "%.2f", turret.getTargetAngle());
            telemetry.addData("Error", "%.2f", turret.getError());
            telemetry.addData("Home Pressed", turret.isHomePressed());

            telemetry.update();
        }
    }
}
