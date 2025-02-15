package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.PhysicsSim.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SimSwerveCmd extends Command {
    private SwerveDrive s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public SimSwerveCmd(
            SwerveDrive s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), .1);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), .1);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), .1);

        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(4.5),
                rotationVal * 10,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}