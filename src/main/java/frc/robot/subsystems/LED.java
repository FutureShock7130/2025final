// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;
    private final Timer timer = new Timer();
    private static LED mInstance = null;

    public static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    /** Creates a new LED. */
    public LED() {
        m_led = new AddressableLED(9);
        m_ledBuffer = new AddressableLEDBuffer(105);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void resetLED(int port, int length) {
        m_led.stop();
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    int counter = 0;

    public void color(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void nocolor() {
        color(0, 0, 0);
    }

    public void blink(int r, int g, int b) {
        timer.start();
        if (timer.get() < 0.1) {
            color(r, g, b);
        } else if (timer.get() < 0.2) {
            color(0, 0, 0);
        } else {
            timer.reset();

        }
    }

    public void marquee(int r, int g, int b) {
        timer.start();
        if (timer.get() < 0.1) {
            for (int i = 0; i < m_ledBuffer.getLength() - 1; i++) {
                if (((int) ((i + counter) / 6)) % 2 == 0)
                    m_ledBuffer.setRGB(i, r, g, b);
                else
                    m_ledBuffer.setRGB(i, 0, 0, 0);
            }
        } else if (timer.get() > 0.2) {
            counter++;
            timer.restart();
        }

        m_led.setData(m_ledBuffer);
    }

    public void rainbowmarquee() {
        timer.start();
        if (timer.get() < 0.1) {
            for (int i = 0; i < m_ledBuffer.getLength() - 1; i++) {
                if ((i + counter) % 7 == 0)
                    m_ledBuffer.setRGB(i, 255, 0, 0);
                if ((i + counter) % 7 == 1)
                    m_ledBuffer.setRGB(i, 255, 100, 0);
                if ((i + counter) % 7 == 2)
                    m_ledBuffer.setRGB(i, 255, 255, 0);
                if ((i + counter) % 7 == 3)
                    m_ledBuffer.setRGB(i, 0, 255, 0);
                if ((i + counter) % 7 == 4)
                    m_ledBuffer.setRGB(i, 0, 127, 255);
                if ((i + counter) % 7 == 5)
                    m_ledBuffer.setRGB(i, 0, 0, 255);
                if ((i + counter) % 7 == 6)
                    m_ledBuffer.setRGB(i, 139, 0, 255);
            }
        } else if (timer.get() < 0.2) {
        } else {
            counter++;
            timer.restart();
        }
        m_led.setData(m_ledBuffer);

    }

    public void rainbowblink() {
        timer.start();
        if (timer.get() < 0.1) {
            color(255, 0, 0);
        } else if (timer.get() < 0.2) {
            color(255, 100, 0);
        } else if (timer.get() < 0.3) {
            color(255, 255, 0);
        } else if (timer.get() < 0.4) {
            color(0, 255, 0);
        } else if (timer.get() < 0.5) {
            color(0, 127, 255);
        } else if (timer.get() < 0.6) {
            color(0, 0, 255);
        } else if (timer.get() < 0.7) {
            color(139, 0, 255);
        } else if (timer.get() < 0.8) {
            timer.reset();

        }
    }

    public void charge(int r,int g,int b,int blink) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, r, g, b);
          m_led.setData(m_ledBuffer);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        for (int j = 0; j < blink; j++) {
          for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
          }
          m_led.setData(m_ledBuffer);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
          for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
          }
          m_led.setData(m_ledBuffer);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
      }
      
    public void chris() {
        timer.start();
        if (timer.get() < 0.1) {
            for (int i = 89; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 255, 255, 0);
            }
            for (int i = 0; i < m_ledBuffer.getLength() - 16; i++) {
                if ((i + counter) % 7 == 0)
                    m_ledBuffer.setRGB(i, 255, 0, 0);
                if ((i + counter) % 7 == 1)
                    m_ledBuffer.setRGB(i, 255, 100, 0);
                if ((i + counter) % 7 == 2)
                    m_ledBuffer.setRGB(i, 255, 255, 0);
                if ((i + counter) % 7 == 3)
                    m_ledBuffer.setRGB(i, 0, 255, 0);
                if ((i + counter) % 7 == 4)
                    m_ledBuffer.setRGB(i, 0, 127, 255);
                if ((i + counter) % 7 == 5)
                    m_ledBuffer.setRGB(i, 0, 0, 255);
                if ((i + counter) % 7 == 6)
                    m_ledBuffer.setRGB(i, 139, 0, 255);
            }
        } else if (timer.get() < 0.2) {
            for (int i = 89; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
        } else {
            counter++;
            timer.restart();
        }
        m_led.setData(m_ledBuffer);

    }

    @Override
    public void periodic() {

    }
}
