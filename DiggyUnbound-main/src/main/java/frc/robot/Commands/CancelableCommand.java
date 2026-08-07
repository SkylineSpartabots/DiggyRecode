package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CancelableCommand extends Command {

    CommandXboxController controller;
    
    public CancelableCommand(CommandXboxController controller) {
        this.controller = controller;
    }
  
    public boolean isFinished() {
      return controller.b().getAsBoolean();
    }
}