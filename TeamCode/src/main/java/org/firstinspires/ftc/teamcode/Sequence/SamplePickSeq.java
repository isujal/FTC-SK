package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SamplePickSeq {
    public static Action SamplePickSeq(Intake intake, Outtake outtake) {
        return new SequentialAction(
                outtake.LifterCommands(Outtake.LifterState.INIT),
                intake.ExtensionCommands(Intake.ExtensionState.FULL),
                intake.IntakeBucketCommands(Intake.IntakeBucketState.PICK),
                new SleepAction(1),
                intake.IntakeRollerCommands(Intake.IntakeRollerState.ON),
                new SleepAction(0.6),
                new ParallelAction(
                        intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                        intake.IntakeGripperCommands(Intake.SampleGripperState.OPEN)
                )


        );
    }
        public static Action Extension(Intake intake, Outtake outtake) {
            return new SequentialAction(
            outtake.LifterCommands(Outtake.LifterState.INIT),
                    intake.ExtensionCommands(Intake.ExtensionState.MAX),
                    intake.IntakeBucketCommands(Intake.IntakeBucketState.PICK),
                    new SleepAction(1),
                    intake.IntakeRollerCommands(Intake.IntakeRollerState.ON),
                    new SleepAction(0.4),
                    new ParallelAction(
                            intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                            intake.IntakeGripperCommands(Intake.SampleGripperState.OPEN)
                    )
            );
        }
}
