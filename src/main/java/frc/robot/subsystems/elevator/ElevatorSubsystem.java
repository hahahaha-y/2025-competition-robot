package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

@Getter
public class ElevatorSubsystem extends SubsystemBase {

    public enum WantedState {
        INTAKE_POSITION,
        INTAKER_AVOID,
        SHOOT_POSITION,
        ZERO,
        IDLE
    }

    public enum SystemState {
        INTAKE_POSITIONING,
        INTAKER_AVOIDING,
        SHOOT_POSITIONING,
        ZEROING,
        IDLING
    }

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;
    private WantedState previousWantedState = WantedState.IDLE;

    private String intakePositionName = "Null";
    private double intakePosition = 0.0;
    private String shootPositionName = "Null";
    private double shootPosition = 0.0;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Elevator/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.ZEROING;
        }

        // set movements based on state
        switch (systemState) {
            case INTAKE_POSITIONING:
                io.setElevatorTarget(intakePosition);
                break;
            case INTAKER_AVOIDING:
                io.setElevatorTarget(RobotConstants.ElevatorConstants.INTAKE_AVOIDING_HEIGHT);
                break;
            case SHOOT_POSITIONING:
                io.setElevatorTarget(shootPosition);
                break;
            case ZEROING:
                io.zeroingElevator();
                break;
            case IDLING:
            default:
                break; //Todo: How to write the idle? motor.setVoltageOut(0)?
        }
    }


    private SystemState handleStateTransition() {
        //Todo: change all the true in the if course into boolean variable after connect with other subsystem
        return switch (wantedState) {
            case ZERO -> {
                if (systemState == SystemState.INTAKE_POSITIONING) {
                    if (true/*isIntakerOff*/){
                        yield SystemState.ZEROING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.INTAKE_POSITIONING;
                }
                if (systemState == SystemState.SHOOT_POSITIONING) {
                    if (true/*isShooterOff*/) {
                        yield SystemState.ZEROING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.INTAKE_POSITIONING;
                }
                if (systemState == SystemState.ZEROING) {
                    wantedState = previousWantedState;
                    yield SystemState.ZEROING;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if(true/*isIntakeNotMoving*/){
                        yield SystemState.ZEROING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.INTAKER_AVOIDING;
                }
                if(io.isCurrentMax(RobotConstants.ElevatorConstants.ELEVATOR_ZEROING_CURRENT)){
                    yield SystemState.IDLING;
                }
                yield SystemState.ZEROING;
            }
            case INTAKE_POSITION -> {
                if (systemState == SystemState.IDLING){
                    if(true/*no things in the shooter*/){
                        yield SystemState.INTAKE_POSITIONING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.IDLING;
                }
                if (systemState == SystemState.SHOOT_POSITIONING) {
                    if (true/*is shooter motor off*/ && true /*no things in the shooter*/) {
                        yield SystemState.INTAKE_POSITIONING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.SHOOT_POSITIONING;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/ && true /*no things in the shooter*/){
                        yield SystemState.INTAKE_POSITIONING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.INTAKER_AVOIDING;
                }
                if(systemState == SystemState.ZEROING){
                    wantedState = previousWantedState;
                    yield SystemState.ZEROING;
                }
                wantedState = previousWantedState;
                yield SystemState.IDLING;
            }
            case INTAKER_AVOID -> {
                if (systemState == SystemState.INTAKE_POSITIONING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.INTAKER_AVOIDING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.INTAKE_POSITIONING;
                }
                if (systemState == SystemState.SHOOT_POSITIONING) {
                    if (true/*is shooter motor off*/) {
                        yield SystemState.INTAKER_AVOIDING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.SHOOT_POSITIONING;
                }
                if(systemState == SystemState.ZEROING){
                    wantedState = previousWantedState;
                    yield SystemState.ZEROING;
                }
                yield SystemState.INTAKER_AVOIDING;
            }
            case SHOOT_POSITION -> {
                if (systemState == SystemState.INTAKE_POSITIONING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.SHOOT_POSITIONING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.INTAKE_POSITIONING;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/){
                        yield SystemState.SHOOT_POSITIONING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.INTAKER_AVOIDING;
                }
                if (systemState == SystemState.ZEROING) {
                    wantedState = previousWantedState;
                    yield SystemState.ZEROING;
                }
                yield SystemState.SHOOT_POSITIONING;
            }
            case IDLE -> {
                if (systemState == SystemState.INTAKE_POSITIONING) {
                    if (true/*is intaker motor off*/){
                        yield SystemState.IDLING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.INTAKE_POSITIONING;
                }
                if (systemState == SystemState.INTAKER_AVOIDING) {
                    if (true/*is intaker come in or out finished*/){
                        yield SystemState.IDLING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.INTAKER_AVOIDING;
                }
                if (systemState == SystemState.ZEROING) {
                    wantedState = previousWantedState;
                    yield SystemState.ZEROING;
                }
                if (systemState == SystemState.SHOOT_POSITIONING) {
                    if (true/*is shooter motor off*/) {
                        yield SystemState.IDLING;
                    }
                    wantedState = previousWantedState;
                    yield SystemState.SHOOT_POSITIONING;
                }
                yield SystemState.IDLING;
            }
            default -> {
                yield SystemState.IDLING;
            }
        };
    }

    public void setELevatorState(WantedState wantedState) {
        previousWantedState = this.wantedState;
        this.wantedState = wantedState;
    }

    public Command setElevatorStateCommand(WantedState wantedState) {
        return new InstantCommand(() -> setELevatorState(wantedState));
    }

    public void setElevatorIntakePosition(String intakePositionName, double intakePosition ) {
        this.intakePositionName = intakePositionName;
        this.intakePosition = intakePosition;
    }

    public Command setElevatorIntakePositionCommand(String intakePositionName , double intakePosition){
        return new InstantCommand(() -> setElevatorIntakePosition(intakePositionName , intakePosition));
    }

    public void setElevatorShootPosition(String shootPositionName,double shootPosition) {
        this.shootPositionName = shootPositionName;
        this.shootPosition = shootPosition;
    }

    public Command setElevatorShootPositionCommand(String shootPositionName,double shootPosition){
        return new InstantCommand(() -> setElevatorShootPosition(shootPositionName,shootPosition));
    }
}
