package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    //TODO ----------------------------ACTUATORS--------------------------------
    public CRServo intakeRoller;
    public Servo specimenGripper, sampleGripper, shoulderR, shoulderL, intakeBucket;
    public DcMotorEx LifterR, LifterL, Extension;

    //TODO -----------------------------SENSORS SUBSYSTEM--------------------------------

    public RevColorSensorV3 colorSensor=null;


    // Static instance to be used across all instances
    private static RobotHardware instance;
    public boolean enabled;
    private HardwareMap hardwareMap;  // Linking to hardware map with robot hardware.


    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;


        //TODO============================= MAPPING INTAKE ACTUATORS ===============================

        intakeRoller = hardwareMap.get(CRServo.class,"Roller");
        specimenGripper = hardwareMap.get(Servo.class, "Specimen");
        intakeBucket = hardwareMap.get(Servo.class, "Bucket");
        shoulderR = hardwareMap.get(Servo.class, "ShoulderR");
        shoulderL = hardwareMap.get(Servo.class, "ShoulderL");
        sampleGripper = hardwareMap.get(Servo.class,"Sample");

        Extension = hardwareMap.get(DcMotorEx.class, "Extension");
        LifterR = hardwareMap.get(DcMotorEx.class, "LifterR");
        LifterL = hardwareMap.get(DcMotorEx.class, "LifterL");


//
//        //TODO============================= MAPPING SENSORS ===============================
        colorSensor=hardwareMap.get(RevColorSensorV3.class,"color");

        //TODO============================= OTHER INITIALIZATION ===============================

        LifterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        LifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LifterR.setDirection(DcMotorSimple.Direction.REVERSE);

        LifterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        LifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LifterL.setDirection(DcMotorSimple.Direction.FORWARD);

        Extension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extension.setDirection(DcMotorSimple.Direction.FORWARD);

        // Add any other initialization code here if needed.
    }
//    public void init(HardwareMap hardwareMap) {
//        s1 = hardwareMap.get(Servo.class, "s1");
//    }

    public void resetEncoder() {
        LifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
