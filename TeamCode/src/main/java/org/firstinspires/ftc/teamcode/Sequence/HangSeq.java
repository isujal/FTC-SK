package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class HangSeq {
    public static Action HangSeq(Intake intake, Outtake outtake) {
        return new SequentialAction(
                outtake.LifterCommands(Outtake.LifterState.HANG)
        );
    }




}
