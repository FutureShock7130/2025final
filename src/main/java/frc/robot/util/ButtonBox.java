package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBox {

    private CommandJoystick buttonBox1;
    private CommandJoystick buttonBox2;

    public ButtonBox(CommandJoystick btn1, CommandJoystick btn2) {
        buttonBox1 = btn1;
        buttonBox2 = btn2;
    }

    public Trigger navA_onTrue() {
        return buttonBox1.axisGreaterThan(1, 0.5);
    }

    public Trigger navB_onTrue() {
        return buttonBox1.button(5);
    }

    public Trigger navC_onTrue() {
        return buttonBox1.button(6);
    }

    public Trigger navD_onTrue() {
        return buttonBox1.button(1);
    }

    public Trigger navE_onTrue() {
        return buttonBox1.button(10);
    }

    public Trigger navF_onTrue() {
        return buttonBox1.button(9);
    }

    public Trigger navG_onTrue() {
        return buttonBox1.button(8);
    }

    public Trigger navH_onTrue() {
        return buttonBox1.button(7);
    }

    public Trigger navI_onTrue() {
        return buttonBox1.button(4);
    }

    public Trigger navJ_onTrue() {
        return buttonBox1.button(3);
    }

    public Trigger navK_onTrue() {
        return buttonBox1.button(2);
    }

    public Trigger navL_onTrue() {
        return buttonBox2.button(3);
    }

    public Trigger eleL1_onTrue() {
        return buttonBox1.button(12);
    }

    public Trigger eleL2_onTrue() {
        return buttonBox1.button(1);
    }

    public Trigger eleL3_onTrue() {
        return buttonBox2.button(4);
    }

    public Trigger eleL4_onTrue() {
        return buttonBox2.button(5);
    }

    public Trigger yellowRight_onTrue() {
        return buttonBox2.button(6);
    }
    
    public Trigger yellowLeft_onTrue() {
        return buttonBox1.button(11);
    }

    public Trigger rightTop_onTrue() {
        return buttonBox2.button(7);
    }

    public Trigger rightMid_onTrue() {
        return buttonBox2.button(9);
    }

    public Trigger rightBottom_onTrue() {
        return buttonBox2.button(8);
    }

    public Trigger leftTop_onTrue() {
        return buttonBox2.button(10);
    }

    public Trigger leftMid_onTrue() {
        return buttonBox2.button(11);
    }

    public Trigger leftBottom_onTrue() {
        return buttonBox2.button(2);
    }
    
}
