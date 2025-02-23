package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SamplePrePickSeq {
    public static Action SamplePrePickSeq(Intake intake, Outtake outtake) {
        return new SequentialAction(
                outtake.ShoulderCommands(Outtake.ShoulderState.INIT),
                outtake.LifterCommands(Outtake.LifterState.INIT),
                intake.IntakeBucketCommands(Intake.IntakeBucketState.SAFE),
                intake.ExtensionCommands(Intake.ExtensionState.FULL),
                intake.IntakeGripperCommands(Intake.SampleGripperState.OPEN)


        );
    }
}
