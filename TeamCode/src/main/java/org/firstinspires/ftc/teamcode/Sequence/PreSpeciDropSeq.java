package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class PreSpeciDropSeq {
    public static Action PreSpeciDropSeq(Intake intake, Outtake outtake) {
        return new SequentialAction(
                outtake.SpecimenGripperCommands(Outtake.SpecimenGripperState.CLOSE),
                outtake.LifterCommands(Outtake.LifterState.PRE_DROP)
        );
    }




}
