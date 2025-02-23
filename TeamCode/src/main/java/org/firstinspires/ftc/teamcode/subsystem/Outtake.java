package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Outtake {

    public static  RobotHardware robot ;  // RobotHardware Object
    public ShoulderState shoulderState = ShoulderState.INIT;
    public LifterState lifterState = LifterState.INIT;
    public SpecimenGripperState specimenGripperState = SpecimenGripperState.INIT;

    public Outtake(RobotHardware robot){
        this.robot = robot;
        updateLifterState(LifterState.INIT);
    }
    public enum ShoulderState {
        INC_VAL,
        HOME,
        INIT,
        PASS,
        DROP,
        PRE_DROP
    }
    public enum LifterState {
        INIT,
        HIGH,
        PICK,
        PRE_DROP,
        DROP,
        PRE_HANG,
        HANG
    }
    public enum SpecimenGripperState {
        INIT,
        OPEN,
        CLOSE,
    }

    public void updateSpecimenGripperState(SpecimenGripperState state) {
        this.specimenGripperState = state;
        double currentGripperState= Globals.SpecimenInit;  // default to off state
        switch (state) {
            case INIT:
                currentGripperState= Globals.SpecimenInit;  // set to on state
                break;
            case OPEN:
                currentGripperState= Globals.SpecimenDrop; // set to off state
                break;
            case CLOSE:
                currentGripperState= Globals.SpecimenPick; // set to release state
                break;
        }
        setSpecimenGripper(currentGripperState);
    }

    public void updateShoulderState(ShoulderState state) {
        this.shoulderState = state;
        double currenShoulderState= Globals.ShoulderPosInit;  // default to off state
        switch (state) {
            case INIT:
                currenShoulderState= Globals.ShoulderPosInit;  // set to on state
                break;
            case PASS:
                currenShoulderState= Globals.ShoulderPass; // set to off state
                break;
            case DROP:
                currenShoulderState= Globals.ShoulderDrop; // set to release state
                break;
            case PRE_DROP:
                currenShoulderState= Globals.ShoulderPreDrop; // set to release state\
                break;
        }
        setShoulder(currenShoulderState);
    }

    public void updateLifterState(LifterState state) {
        this.lifterState = state;

        switch (state) {
            case INIT:
                extendTo(Globals.LifterInit);
                break;
            case HIGH:
                extendTo(Globals.LifterHigh);
                break;
            case PRE_DROP:
                extendTo(Globals.LifterSpecimenPreDrop);
                break;
            case DROP:
                extendTo(Globals.LifterSpecimenDrop);
                break;
            case PICK:
                extendTo(Globals.LifterSpecimenPick);
                break;
            case HANG:
                extendTo(Globals.LifterHangDone);
                break;
            case PRE_HANG:
                extendTo(Globals.LifterPreHang);
                break;
        }


    }

    public InstantAction SpecimenGripperCommands(SpecimenGripperState state) {
        return new InstantAction(() -> updateSpecimenGripperState(state));
    }
    public InstantAction ShoulderCommands(ShoulderState state) {
        return new InstantAction(() -> updateShoulderState(state));
    }
    public InstantAction LifterCommands(LifterState state) {
        return new InstantAction(() -> updateLifterState(state));
    }
    public void setIntakeRoller(double power){
        robot.intakeRoller.setPower(power);
    }
    public void setSpecimenGripper(double pos){
        robot.specimenGripper.setPosition(pos);
    }
    public void setShoulder(double pos){
        robot.shoulderL.setPosition(1 - pos);
        robot.shoulderR.setPosition(pos);
    }

    public static void extendTo(int targetPosition) {
        robot.LifterL.setTargetPosition(targetPosition);
        robot.LifterR.setTargetPosition(targetPosition);

        robot.LifterL.setTargetPositionTolerance(10);
        robot.LifterR.setTargetPositionTolerance(10);

        robot.LifterL.setPower(Globals.ExtendPower);
        robot.LifterR.setPower(Globals.ExtendPower);

        robot.LifterL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.LifterR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}
