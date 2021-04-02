package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class ArcadeDrive extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final RomiDrivetrain m_subsystem;
    private final Joystick m_drivejoy;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArcadeDrive(RomiDrivetrain subsystem, Joystick drivejoy) {
        m_subsystem = subsystem;
        m_drivejoy = drivejoy;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.arcadeDrive(applyCurve(m_drivejoy.getX())*(1-m_drivejoy.getThrottle()), applyCurve(m_drivejoy.getY())*(1-m_drivejoy.getThrottle()));
    }
    public double applyCurve(double val) {
        return val*val*val;
    }
    public double applyCurve(double val, double f){
        return Math.sin(val*val*val);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }   
}
