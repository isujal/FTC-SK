package org.firstinspires.ftc.teamcode.AutoSequences;//package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.instantcommands.IntakeGripperCommand;
import org.firstinspires.ftc.teamcode.instantcommands.IntakeRollerCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SampleDropSeq {
    public SampleDropSeq(Intake intake, Outtake outtake) {
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                outtake.ShoulderCommands(Outtake.ShoulderState.DROP),
                                intake.IntakeBucketCommands(Intake.IntakeBucketState.TRANSFER)
                        ),
                        new SleepAction(0.5),
                        intake.IntakeGripperCommands(Intake.SampleGripperState.OPEN),
                        new SleepAction(0.2),
                        intake.IntakeGripperCommands(Intake.SampleGripperState.INIT),
                        intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                        outtake.LifterCommands(Outtake.LifterState.INIT),
                        outtake.ShoulderCommands(Outtake.ShoulderState.INIT),
                        intake.ExtensionCommands(Intake.ExtensionState.FULL)                      )                          );
    }

}










