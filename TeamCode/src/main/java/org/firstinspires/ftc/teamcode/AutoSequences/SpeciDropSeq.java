package org.firstinspires.ftc.teamcode.AutoSequences;//package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SpeciDropSeq {
    public SpeciDropSeq(Intake intake, Outtake outtake) {
        Actions.runBlocking(
                new SequentialAction(
                        outtake.LifterCommands(Outtake.LifterState.DROP),
                        outtake.SpecimenGripperCommands(Outtake.SpecimenGripperState.CLOSE),
                        new SleepAction(0.5),
                        outtake.SpecimenGripperCommands(Outtake.SpecimenGripperState.OPEN)



                        ));
    }

}


