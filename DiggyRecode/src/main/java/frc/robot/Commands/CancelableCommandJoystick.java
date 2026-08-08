package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CancelableCommandJoystick extends Command {

    CommandXboxController controller;
    
    public CancelableCommandJoystick(CommandXboxController controller) {
        this.controller = controller;
    }
  
    public boolean isFinished() {
      return Math.abs(controller.getLeftX()) > 0.075 || Math.abs(controller.getLeftY()) > 0.075;
    }
}