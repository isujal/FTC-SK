package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public static double shoulderPos = 0;

    //TODO----------------------------- BUCKET--------------------------

    public static double BucketSafe = 0.5;  //0.45  (torque dual mode servo)
    public static double Bucketpick = 0.96;  //0.17
    public static double Bucketpass = 0.01;  //1
    public static double BucketAutoPick = 0.75;  //1
    public static double BucketTransfer = 0.5;  //1

    public static double BucketInc = 0;
    public static double BucketDec = 0;

    //TODO----------------------------- ROLLER---------------------------

    public static double INTAKE_ON = 1;
    public static double INTAKE_OFF = 0;
    public static double INTAKE_RELEASE = -1;

    //TODO------------------------- SAMPLE GRIPPER------------------------

    public static double SampleInit = 0.8;
    public static double SamplePick = 0.45;
    public static double SamplePass = 0.8;
    public static double SampleInc = 0;
    public static double SampleDec = 0;

    //TODO--------------------------- SHOULDERS----------------------------

    public static double ShoulderPosInit =0.02;
    public static double ShoulderPass = 0.07;
    public static double ShoulderPreDrop = 0.5;
    public static double ShoulderDrop = 0.6;
    public static double ShoulderInc = 0;
    public static double ShoulderDec = 0;


    //TODO+---------------------- SPECIMEN GRIPPER------------------------------

    public static double SpecimenInit = 0.8;
    public static double SpecimenPick = 0.38;
    public static double SpecimenDrop = 0.5;
    public static double SpecimenInc = 0;
    public static double SpecimenDec = 0;


    //TODO --------------------------LIFTER-----------------------------------
    public static int LifterInit = 0;
    public static int LifterHigh = 3300;
    public static int LifterLow = 0;
    public static int LifterHangDone = 1850;
    public static int LifterPreHang = 3000;
    public static double LifterPower = 1;
    public static int LifterSpecimenPick = 1;
    public static int LifterSpecimenDrop = 1000;
    public static int LifterSpecimenPreDrop = 2200;
    public static int LifterIncr = 30;
    public static int LifterDecr = 30;

    //TODO --------------------------X-EXTENSIONS-----------------------------------
    public static int ExtensionInit = 0;
    public static int ExtensionHalf = 0;
    public static int ExtensionAutoPrePick = 280;
    public static int ExtensionAutoPrePickTwo = 500;
    public static int ExtensionFull = 300;
    public static int ExtensionMax = 500;
    public static double ExtendPower = 1;
    public static int ExtensionInc = 30;
    public static int ExtensionDec = 200;

//
}
//
//
