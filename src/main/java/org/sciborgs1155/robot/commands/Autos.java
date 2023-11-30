package org.sciborgs1155.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.sciborgs1155.robot.drive.*;

public final class Autos implements Sendable {

  private final SendableChooser<Command> chooser;

  public Autos(Drive drive) {
    chooser = AutoBuilder.buildAutoChooser();

    NamedCommands.registerCommand("lock", drive.lock());
    NamedCommands.registerCommand("stop", drive.stop());
    // NamedCommands.registerCommand("intake", intake.intake());
    // NamedCommands.registerCommand("Score_H", intake.score(high));
    // NamedCommands.registerCommand("Score_M", intake.score(mid));
    // NamedCommands.registerCommand("Score_L", intake.score(low));

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }
}
