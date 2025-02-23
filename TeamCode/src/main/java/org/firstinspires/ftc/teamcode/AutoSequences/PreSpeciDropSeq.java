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

public class PreSpeciDropSeq {
    public PreSpeciDropSeq(Intake intake, Outtake outtake) {
        Actions.runBlocking(
                new SequentialAction(
                        outtake.SpecimenGripperCommands(Outtake.SpecimenGripperState.CLOSE),
                        outtake.LifterCommands(Outtake.LifterState.PRE_DROP)
                )                          );
    }

}










