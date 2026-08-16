package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricDrivetrain {

    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;

    private final IMU imu;

    private double driveSpeedMultiplier = 0.8;

    public FieldCentricDrivetrain(HardwareMap hardwareMap) {

        // ----- Motors -----
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // ----- IMU -----
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void driveFieldRelative(double forward, double right, double rotate) {

        // Convert joystick vector to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Rotate by the opposite of the robot's heading
        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        // Convert back into robot-relative X/Y
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        driveRobotRelative(newForward, newRight, rotate);
    }

    public void driveRobotRelative(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        // Normalize powers if any magnitude is over 1
        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(
                driveSpeedMultiplier * frontLeftPower / maxPower
        );

        frontRightDrive.setPower(
                driveSpeedMultiplier * frontRightPower / maxPower
        );

        backLeftDrive.setPower(
                driveSpeedMultiplier * backLeftPower / maxPower
        );

        backRightDrive.setPower(
                driveSpeedMultiplier * backRightPower / maxPower
        );
    }

    public void setDriveSpeedMultiplier(double multiplier) {
        driveSpeedMultiplier = multiplier;
    }

    public void resetYaw() {
        imu.resetYaw();
    }
}