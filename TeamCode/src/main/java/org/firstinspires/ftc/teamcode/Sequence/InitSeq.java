package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class InitSeq {
    public static Action InitSeqence(Intake intake, Outtake outtake) {
        return new SequentialAction(
                outtake.SpecimenGripperCommands(Outtake.SpecimenGripperState.INIT),
                intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                outtake.LifterCommands(Outtake.LifterState.INIT),
                outtake.ShoulderCommands(Outtake.ShoulderState.PASS),
                intake.ExtensionCommands(Intake.ExtensionState.INIT),
                new SleepAction(0.5),
                intake.IntakeBucketCommands(Intake.IntakeBucketState.PASS)
        );
    }




}
