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

public class PreSampleIntakeSeq {
    public PreSampleIntakeSeq(Intake intake, Outtake outtake) {
        Actions.runBlocking(
                new SequentialAction(
                        outtake.ShoulderCommands(Outtake.ShoulderState.INIT),
                        outtake.LifterCommands(Outtake.LifterState.INIT),
                        intake.IntakeBucketCommands(Intake.IntakeBucketState.SAFE),
                        intake.ExtensionCommands(Intake.ExtensionState.FULL),
                        intake.IntakeGripperCommands(Intake.SampleGripperState.OPEN)
                )                                );
    }

}


