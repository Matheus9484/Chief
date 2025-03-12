package frc.robot.commands.OdometriaAlinhamento;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlinhamentoOdometria extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private Command goToAprilTag;
    Pose2d targetPose;
    int targetAprilTag;
    CommandXboxController xbox;
    boolean finalizar;
    Rotation2d angulo;
    boolean alianca;

    //carrega as posições dos AprilTags.
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeAndyMark
    );


    public AlinhamentoOdometria(SwerveSubsystem swerveSubsystem, int targetAprilTag, CommandXboxController xbox, Rotation2d angulo) {
        this.targetAprilTag = targetAprilTag;
        this.xbox = xbox;
        this.swerveSubsystem = swerveSubsystem;
        this.angulo = angulo;

        addRequirements(swerveSubsystem);
    }
 
    //comando que inicializa o alinhamento.
    @Override
    public void initialize() {
        // Define a posição alvo baseada no AprilTag.
        finalizar = false;
        Transform2d robotOffset = new Transform2d(new Translation2d(0.75, 0), angulo);
        targetPose = getAprilTagPose(targetAprilTag, robotOffset);

        // Cria o comando de movimentação e armazena ele.
        goToAprilTag = swerveSubsystem.driveToPose(targetPose);
        goToAprilTag.initialize(); //roda o comando.
    }

    @Override
    public void execute() {
        // Mantém executando enquanto o botão estiver pressionado.
        if(xbox.button(3).getAsBoolean()){
            finalizar = true;
        }
      
        goToAprilTag.execute();
        swerveSubsystem.visao = false;
    }

    @Override
    public void end(boolean interrupted) {
        // Para o robô corretamente quando o botão é solto.
        goToAprilTag.end(interrupted);
        swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
        swerveSubsystem.visao = true;
        finalizar = true;

    }

    //condição para ele parar quando estiver chegando perto.
    @Override
    public boolean isFinished() {
      Pose2d currentPose = swerveSubsystem.getPose(); // Obtém a posição atual do robô.
      double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
  
      return distance < 0.05 || finalizar; 
    }

    //pega a posição do AprilTag.
    public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
        Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
        }
    }
}
