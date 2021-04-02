package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.CheesyDriveHelper;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.RomiDrivetrain;

public class ChezzyDrive extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final RomiDrivetrain m_subsystem;
    private final Joystick m_drivejoy;
    private final CheesyDriveHelper chezz = new CheesyDriveHelper();
    private final double twistQuickturnAmount = 0.5;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ChezzyDrive(RomiDrivetrain subsystem, Joystick drivejoy) {
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
        DriveSignal drv = chezz.cheesyDrive(m_drivejoy.getY(), -m_drivejoy.getX(), m_drivejoy.getRawButton(1), false);
        double inchesPerSecondCap = 20 * (1 - m_drivejoy.getThrottle());
        SmartDashboard.putNumber("speed range in/s", inchesPerSecondCap);
        double left = drv.getLeft();
        left -= m_drivejoy.getTwist() * twistQuickturnAmount;
        left *= inchesPerSecondCap;

        double right = -drv.getRight();
        right -= m_drivejoy.getTwist() * twistQuickturnAmount;
        right *= inchesPerSecondCap;

        m_subsystem.velocityDrive(left, right);
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
