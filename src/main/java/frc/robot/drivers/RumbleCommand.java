package frc.robot.drivers;

import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleCommand extends Command {
    private final GenericHID[] hid;
    private final Timer timer = new Timer();
    private final Measure<edu.wpi.first.units.TimeUnit> rumbleTime;

    public RumbleCommand(Measure<edu.wpi.first.units.TimeUnit> seconds, GenericHID... hid) {
        this.hid = hid;
        this.rumbleTime = seconds;
    }

    @Override
    public void initialize() {
        timer.restart();
        for (var i : hid) {
            i.setRumble(GenericHID.RumbleType.kBothRumble, 1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (var i : hid) {
            i.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(rumbleTime.magnitude());
    }
}