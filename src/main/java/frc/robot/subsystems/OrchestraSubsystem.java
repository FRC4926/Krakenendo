// package frc.robot.subsystems;

// import com.ctre.phoenix6.Orchestra;
// import com.ctre.phoenix6.configs.AudioConfigs;

// import edu.wpi.first.wpilibj2.command.Subsystem;

// public class OrchestraSubsystem extends Orchestra implements Subsystem {
//     public OrchestraSubsystem(CommandSwerveDrivetrain drivetrain, String chrp_path) {
//         super();

//         AudioConfigs ac = new AudioConfigs();
//         ac.AllowMusicDurDisable = true;
//         drivetrain.setAllMotorConfigs(ac);

//         drivetrain.addAllMotorsToOrchestra(this);
//         loadMusic(chrp_path);
//     }
// }
