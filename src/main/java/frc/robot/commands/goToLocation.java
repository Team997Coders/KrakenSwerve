package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import java.util.ArrayList;
import java.util.List;

public class goToLocation extends Command {

    private Drivebase drivebase;
    private Pose2d goalPose;
    private List<Pose2d> poses;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 2);
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(60, 60);

    private final ProfiledPIDController xController = new ProfiledPIDController(5, 0, .2, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(5, 0, .2, Y_CONSTRAINTS);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, THETA_CONSTRAINTS);

    @SuppressWarnings("unused")
    private double xStart = 0;

    @SuppressWarnings("unused")
    private double yStart = 0;

    @SuppressWarnings("unused")
    private double thetaStart = 0;

    public goToLocation(Drivebase drivebase, ArrayList<Pose2d> poses) {
        this.drivebase = drivebase;
        this.poses = poses;

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(Units.degreesToRadians(2));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivebase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double bestDistance = 99999;
        Pose2d bestPose = new Pose2d();
        Pose2d robotPose = this.drivebase.getPose();
        List<Double> distances = new ArrayList<>();

        for (int i = 0; i < poses.size(); i++) {

            double distance = Math.sqrt(Math.pow(robotPose.getX() - poses.get(i).getX(), 2)
                    + Math.pow(robotPose.getY() - poses.get(i).getY(), 2));
            distances.add(distance);

            if (distance < bestDistance) {
                bestPose = poses.get(i);
                bestDistance = distance;
            }
        }

        this.goalPose = bestPose;

        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        thetaController.reset(robotPose.getRotation().getRadians());

        xStart = robotPose.getX();
        yStart = robotPose.getY();
        thetaStart = robotPose.getRotation().getRadians();

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        thetaController.setGoal(goalPose.getRotation().getRadians());
    }

    public double xSpeed;
    public double ySpeed;
    public double thetaSpeed;
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d robotPose = drivebase.getPose();

        xSpeed = xController.calculate(robotPose.getX());
        ySpeed = yController.calculate(robotPose.getY());
        thetaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());

        if (xController.atGoal()) {
            xSpeed = 0;
        }
        if (yController.atGoal()) {
            ySpeed = 0;
        }
        if (thetaController.atGoal()) {
            thetaSpeed = 0;
        }

        if (xSpeed == 0 && ySpeed == 0 && thetaSpeed == 0) {
            this.cancel();
        }

        double angle = goalPose.getRotation().getRadians();
        drivebase.drive(new ChassisSpeeds(
                xSpeed * Math.cos(angle) + ySpeed * Math.sin(angle),
                ySpeed * Math.cos(-angle) + xSpeed * Math.sin(-angle),
                thetaSpeed));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean finished = false;
        if (xController.atGoal() && yController.atGoal() && thetaController.atGoal()) {
            finished = true;
        } else {
            finished = false;
        }

        return finished;
    }
}