package org.firstinspires.ftc.teamcode.AutoSequences;//package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SpeciFinalInitSeq {
    public SpeciFinalInitSeq(Intake intake, Outtake outtake) {
        Actions.runBlocking(
                new SequentialAction(
                        intake.IntakeGripperCommands(Intake.SampleGripperState.INIT),
                        intake.IntakeBucketCommands(Intake.IntakeBucketState.PASS),
                        intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                        outtake.LifterCommands(Outtake.LifterState.INIT),
                        outtake.ShoulderCommands(Outtake.ShoulderState.PASS),
                        intake.ExtensionCommands(Intake.ExtensionState.INIT),
                        outtake.SpecimenGripperCommands(Outtake.SpecimenGripperState.INIT)
                )                          );
    }

}










