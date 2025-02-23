package org.firstinspires.ftc.teamcode.instantcommands;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class LifterCommand {

    public LifterCommand(Outtake outtake, Outtake.LifterState state) {
        // Use Actions.runBlocking to execute the command
        Actions.runBlocking(new SequentialAction(
                new InstantAction( () -> outtake.updateLifterState(state))
        ));
    }
}
