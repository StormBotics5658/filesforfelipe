package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
public class drive_command extends Command {
    private final SwerveSubsystem m_SwerveSubsystem;
    private double mx = 0;
    private double my = 0;
    private double mr = 0;
    public drive_command(
        SwerveSubsystem c_SwerveSubsystem,
        double cx,
        double cy,
        double cr
        
    )
    {
        this.m_SwerveSubsystem = c_SwerveSubsystem;
        this.mx = cx;
        this.my = cy;
        this.mr = cr;
        addRequirements(m_SwerveSubsystem);
    
    }
    @Override
    public void execute() {
   //     m_SwerveSubsystem.
    }
}
