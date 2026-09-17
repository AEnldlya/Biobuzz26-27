package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** RED alliance AUTO; the routine itself is in AutoBase. */
@Autonomous(name = "RED AUTO", group = "Biobuzz", preselectTeleOp = "Biobuzz TeleOp")
public class RedAuto extends AutoBase {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
