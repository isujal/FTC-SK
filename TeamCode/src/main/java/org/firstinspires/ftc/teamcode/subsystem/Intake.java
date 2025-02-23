package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Intake {

    public static  RobotHardware robot;  // RobotHardware Object
    public IntakeRollerState rollerState = IntakeRollerState.OFF;
    public SampleGripperState sampleGripperState = SampleGripperState.INIT;
    public IntakeBucketState bucketState = IntakeBucketState.SAFE;
    public ExtensionState extensionState = ExtensionState.INIT;

    public Intake(RobotHardware robot){
        this.robot = robot;
        updateExtensionState(ExtensionState.INIT);
    }
    public enum ExtensionState {
        INC_VAL,
        HOME,
        INIT,
        AUTO_PRE_PICK,
        AUTO_PRE_PICK_TWO,
        SAFE,
        HALF,
        FULL,
        MAX,
        LAST
    }
    public enum IntakeRollerState {
        ON,
        OFF,
        RELEASE
    }
    public enum SampleGripperState {
        INIT,
        OPEN,
        CLOSE,
    }
    public enum IntakeBucketState {
        SAFE,
        PICK,
        PASS,
        AUTO_PICK,
        TRANSFER
    }

    public void updateSampleGripperState(SampleGripperState state) {
        this.sampleGripperState = state;
        double currentGripperState= Globals.SampleInit;  // default to off state
        switch (state) {
            case INIT:
                currentGripperState= Globals.SampleInit;  // set to on state
                break;
            case OPEN:
                currentGripperState= Globals.SamplePass; // set to off state
                break;
            case CLOSE:
                currentGripperState= Globals.SamplePick; // set to release state
                break;
        }
        setIntakeGripper(currentGripperState);
    }

    public void updateState(IntakeRollerState state) {
        this.rollerState=state;
        double currentRollerState= Globals.INTAKE_OFF;  // default to off state
        switch (state) {
            case ON:
                currentRollerState= Globals.INTAKE_ON;  // set to on state
                break;
            case OFF:
                currentRollerState= Globals.INTAKE_OFF; // set to off state
                break;
            case RELEASE:
                currentRollerState= Globals.INTAKE_RELEASE; // set to release state
                break;
        }
        setIntakeRoller(currentRollerState);
    }

    public void updateBucketState(IntakeBucketState state) {
        this.bucketState=state;
        double currentBucketState= Globals.BucketSafe;  // default to off state
        switch (state) {
            case PICK:
                currentBucketState= Globals.Bucketpick;  // set to on state
                break;
            case SAFE:
                currentBucketState= Globals.BucketSafe; // set to off state
                break;
            case PASS:
                currentBucketState= Globals.Bucketpass; // set to release state
                break;
            case AUTO_PICK:
                currentBucketState= Globals.BucketAutoPick; // set to release state
                break;
            case TRANSFER:
                currentBucketState= Globals.BucketTransfer; // set to release state
                break;
        }
        setIntakeBucket(currentBucketState);
    }
    public void updateExtensionState(ExtensionState state) {
        this.extensionState = state;
        switch (state) {
            case INIT:
                extendTo(Globals.ExtensionInit);
                break;

            case HALF:
                extendTo(Globals.ExtensionHalf);
                break;
            case AUTO_PRE_PICK:
                extendTo(Globals.ExtensionAutoPrePick);
                break;
            case AUTO_PRE_PICK_TWO:
                extendTo(Globals.ExtensionAutoPrePickTwo);
                break;

            case FULL:
                extendTo(Globals.ExtensionFull);
                break;
            case MAX:
                extendTo(Globals.ExtensionMax);
                break;
            case LAST:
                extendTo(Globals.ExtensionDec);
                break;
        }
    }

    public InstantAction IntakeRollerCommands(IntakeRollerState state) {
        return new InstantAction(() -> updateState(state));
    }
    public InstantAction IntakeGripperCommands(SampleGripperState state) {
        return new InstantAction(() -> updateSampleGripperState(state));
    }
    public InstantAction IntakeBucketCommands(IntakeBucketState state) {
        return new InstantAction(() -> updateBucketState(state));
    }
    public InstantAction ExtensionCommands(ExtensionState state) {
        return new InstantAction(() -> updateExtensionState(state));
    }
    public void setIntakeRoller(double power){
        robot.intakeRoller.setPower(power);
    }
    public void setIntakeGripper(double pos){
        robot.sampleGripper.setPosition(pos);
    }
    public void setIntakeBucket(double pos){
        robot.intakeBucket.setPosition(pos);
    }

    public static void extendTo(int targetPosition) {
        robot.Extension.setTargetPosition(targetPosition);
        robot.Extension.setTargetPositionTolerance(10);
        robot.Extension.setPower(Globals.ExtendPower);
        robot.Extension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}
