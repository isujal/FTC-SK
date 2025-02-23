package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SamplePickAgainSeq {
    public static Action SamplePickAgainSeq(Intake intake, Outtake outtake) {
        return new SequentialAction(
                intake.IntakeRollerCommands(Intake.IntakeRollerState.ON),
                new SleepAction(0.3),
                        intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF)


        );
    }
}
