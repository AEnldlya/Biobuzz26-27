package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** BLUE alliance AUTO; the routine itself is in AutoBase. */
@Autonomous(name = "BLUE AUTO", group = "Biobuzz", preselectTeleOp = "Biobuzz TeleOp")
public class BlueAuto extends AutoBase {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
