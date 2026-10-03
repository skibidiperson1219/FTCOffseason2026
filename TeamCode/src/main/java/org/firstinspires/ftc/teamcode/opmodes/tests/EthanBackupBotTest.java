package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EthanBackupBotTest {
    private DcMotor LeftMotor; //left motor
    private DcMotor RightMotor; //right motor
    public void setLeftMotorSpeed(double speed) {
        LeftMotor.setPower(1);
        RightMotor.setPower(1);
    }
    private Servo servoPos;
    private CRServo servoCR;

    public void init(HardwareMap hwMap) {
        LeftMotor = hwMap.get(DcMotor.class,"LeftMotor");
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor = hwMap.get(DcMotor.class,"RightMotor");
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servoPos = hwMap.get(Servo.class,"Servo");
        servoCR = hwMap.get(CRServo.class, "CRServo");
    }
}

