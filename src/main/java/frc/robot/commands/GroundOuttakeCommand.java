package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

import static frc.robot.RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS;

public class GroundOuttakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public GroundOuttakeCommand(IntakeSubsystem intakeSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(intakeSubsystem, endEffectorArmSubsystem, elevatorSubsystem);
    }

    @Override
    public void execute() {
        //TODO elevator may not need to home
        if (elevatorSubsystem.getIo().isNearExtension(RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS.get())) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.OUTTAKE);
            endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.CORAL_OUTTAKE);
        } else {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_WITHOUT_ROLL);
        }
        elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOLD);
        elevatorSubsystem.setElevatorPosition(HOME_EXTENSION_METERS.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
