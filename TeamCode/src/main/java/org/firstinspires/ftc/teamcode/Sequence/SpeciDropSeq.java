package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SpeciDropSeq {
    public static Action SpeciDropSeq(Intake intake, Outtake outtake) {
        return new SequentialAction(
                new ParallelAction(
                        outtake.SpecimenGripperCommands(Outtake.SpecimenGripperState.CLOSE),
                        intake.IntakeBucketCommands(Intake.IntakeBucketState.TRANSFER),
                        intake.IntakeGripperCommands(Intake.SampleGripperState.INIT)

                        ),
                outtake.LifterCommands(Outtake.LifterState.DROP),
                new SleepAction(0.5),
                outtake.SpecimenGripperCommands(Outtake.SpecimenGripperState.OPEN)
                );
    }




}
