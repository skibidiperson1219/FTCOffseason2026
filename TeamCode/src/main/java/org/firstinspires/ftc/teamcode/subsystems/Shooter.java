package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    public enum ShooterState {
        IDLE, SPINNING_UP, READY, FEEDING
    }
    private ShooterState state = ShooterState.IDLE;
    private final DcMotorEx flywheelMotorLeft;
    private final DcMotorEx flywheelMotorRight;
    private final Servo hoodServo;
    public final Servo gate;
    private final RGB stateLight;
    private final ElapsedTime feedTimer = new ElapsedTime();


    private double targetVelocity = 0;
    private double velocityTolerance = 50;

    public double gateClosed = 0.0;
    public double gateOpen = 0.5;
    private double feedTime = 1.0;
    private double gateOpenDelay = 0.25;

    private double P = 0;
    private double F = 0;

    public Shooter(HardwareMap hardwareMap) {
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        gate = hardwareMap.get(Servo.class, "gate");
        Servo rgbServo = hardwareMap.get(Servo.class, "rgb2");

        flywheelMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorConfigurationType leftType = flywheelMotorLeft.getMotorType().clone();
        leftType.setAchieveableMaxRPMFraction(1.0);
        flywheelMotorLeft.setMotorType(leftType);

        MotorConfigurationType rightType = flywheelMotorRight.getMotorType().clone();
        rightType.setAchieveableMaxRPMFraction(1.0);
        flywheelMotorRight.setMotorType(rightType);

        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setPIDF(40, 12.64);
        gate.setPosition(gateClosed);
        stateLight = new RGB(rgbServo);
    }

    public void setPIDF(double p, double f) {
        P = p;
        F = f;

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheelMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void requestSpinUp(double velocity) {
        targetVelocity = velocity;
        state = ShooterState.SPINNING_UP;
    }
    public void requestStop() {
        state = ShooterState.IDLE;
    }
    public void requestFeed() {
        if (state == ShooterState.READY) {
            gate.setPosition(gateOpen);
            feedTimer.reset();
            state = ShooterState.FEEDING;
        }
    }
    public void update() {
        switch (state) {
            case IDLE:
                flywheelMotorLeft.setPower(0);
                flywheelMotorRight.setPower(0);
                gate.setPosition(gateClosed);
                stateLight.blue();
                break;

            case SPINNING_UP:
                flywheelMotorLeft.setVelocity(targetVelocity);
                flywheelMotorRight.setVelocity(targetVelocity);
                gate.setPosition(gateClosed);
                stateLight.azure();

                if (atSpeed()) {
                    state = ShooterState.READY;
                }
                break;

            case READY:
                flywheelMotorLeft.setVelocity(targetVelocity);
                flywheelMotorRight.setVelocity(targetVelocity);
                gate.setPosition(gateClosed);
                stateLight.green();

                // if speed drops, go back to spinning up
                if (!atSpeed()) {
                    state = ShooterState.SPINNING_UP;
                }
                break;

            case FEEDING:
                flywheelMotorLeft.setVelocity(targetVelocity);
                flywheelMotorRight.setVelocity(targetVelocity);
                gate.setPosition(gateOpen);
                stateLight.orange();
                
                if (feedTimer.seconds() >= feedTime) {
                    gate.setPosition(gateClosed);

                    if (atSpeed()) {
                        state = ShooterState.READY;
                    } else {
                        state = ShooterState.SPINNING_UP;
                    }
                }
                break;
        }
    }

    public boolean shouldRunTransfer() {
        return state == ShooterState.FEEDING && feedTimer.seconds() >= gateOpenDelay;
    }

    public boolean atSpeed() {
        return Math.abs(targetVelocity - getAverageVelocity()) <= velocityTolerance;
    }

    public double getLeftVelocity() {
        return flywheelMotorLeft.getVelocity();
    }

    public double getRightVelocity() {
        return flywheelMotorRight.getVelocity();
    }

    public double getAverageVelocity() {
        return (Math.abs(getLeftVelocity()) + Math.abs(getRightVelocity())) / 2.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public ShooterState getState() {
        return state;
    }

    public double getFlywheelSpeed(double distance) {
        return 7.24201 * distance + 922.48402;

    }

    public double getHoodAngle(double distance) {
        return 0.0000014368 * distance * distance * distance
                - 0.000362971 * distance * distance
                + 0.0309257 * distance
                - 0.459372;
    }

    public void aimForDistance(double distance) {

        double velocity = getFlywheelSpeed(distance);
        double hoodPos = getHoodAngle(distance);

        // clamp between 1200 and 2000
        velocity = Math.max(1200, Math.min(2000, velocity));
        // clamp between 0.33 and 0.44
        hoodPos = Math.max(0.33, Math.min(0.44, hoodPos));

        setTargetVelocity(velocity);
        hoodServo.setPosition(hoodPos);
    }
}
