package org.firstinspires.ftc.teamcode.Utils.utilities;

public class ToggleUpOrDown { // TODO: Optimize the shit out of this please

    public boolean ToggleUp;
    public boolean ToggleDown;

    public int OffsetTargetPosition;
    public double UpIncrement;
    public double DownIncrement;

    public ToggleUpOrDown(double upIncrement, double downIncrement, int targetPosition){
        this.UpIncrement = upIncrement;
        this.DownIncrement = downIncrement;
        this.OffsetTargetPosition = targetPosition;
    }

    public void upToggle (boolean Btn){
        if (Btn) {
            if (!ToggleUp) {
                ToggleUp = true;
                // this logic runs when you press the button without releasing it
                OffsetTargetPosition += UpIncrement;
            }
        }
        else {
            ToggleUp = false;
        }
    }

    public void downToggle (boolean Btn){
        if (Btn) {
            if (!ToggleDown) {
                ToggleDown = true;
                OffsetTargetPosition -= DownIncrement;
            }
        }
        else {
            ToggleDown = false;
        }
    }

    public void clearOffset(int target){
        OffsetTargetPosition = target;
    }

}
